/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "KrakenEffectorComponent.h"
#include "ApplePickingNotifications.h"
#include "PickingStructs.h"
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <LmbrCentral/Shape/BoxShapeComponentBus.h>

namespace AppleKraken
{
    namespace DebugStateTransit
    {
        // TODO - this is a debug space for a stub implementation. Proper: a state transition machine with lambdas.
        struct HashTransition
        {
            size_t operator()(const std::pair<EffectorState, EffectorState>& p) const
            {
                int16_t first = static_cast<int16_t>(p.first);
                int16_t second = static_cast<int16_t>(p.second);
                size_t combined = (size_t)first << 16 | second;
                return combined;
            }
        };

        // <startState, targetState>, debugTime
        using TransitionMap = AZStd::unordered_map<StateTransition, float, HashTransition>;

        TransitionMap GetTransitionMap()
        {
            static const TransitionMap tm = {
                { std::make_pair(EffectorState::IDLE, EffectorState::PREPARED), 0.1f },
                { std::make_pair(EffectorState::PREPARED, EffectorState::PICKING), 0.05f },
                { std::make_pair(EffectorState::PICKING, EffectorState::RETRIEVING), 0.1f },
                { std::make_pair(EffectorState::RETRIEVING, EffectorState::PREPARED), 0.1f },
                { std::make_pair(EffectorState::PREPARED, EffectorState::IDLE), 0.05f },
            };
            return tm;
        }

        AZStd::string StateTransitionString(EffectorState current, EffectorState next)
        {
            return AZStd::string::format("state transition %d -> %d\n", static_cast<int>(current), static_cast<int>(next));
        }

        float GetStateTransitTime(EffectorState currentState, EffectorState targetState)
        {
            auto transitions = GetTransitionMap();
            auto key = std::make_pair(currentState, targetState);
            if (transitions.contains(key))
            {
                return GetTransitionMap().at(key);
            }

            // Invalid state transitions detected elsewhere
            return 0.0f;
        }
    } // namespace DebugStateTransit

    void KrakenEffectorComponent::Activate()
    {
        ApplePickingRequestBus::Handler::BusConnect(GetEntityId());
        AZ::TickBus::Handler::BusConnect();
    }

    void KrakenEffectorComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        ApplePickingRequestBus::Handler::BusDisconnect();
    }

    void KrakenEffectorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<KrakenEffectorComponent, AZ::Component>()->Version(1)->Field(
                "ReachEntity", &KrakenEffectorComponent::m_reachEntity);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<KrakenEffectorComponent>("Kraken Effector", "Manipulator component for picking apples")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &KrakenEffectorComponent::m_reachEntity,
                        "Kraken Reach entity",
                        "Kraken entity with box shape to set reach area")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true);
            }
        }
    }

    bool KrakenEffectorComponent::IsTransitionValid(EffectorState targetState) const
    {
        auto transitionKey = std::make_pair(m_effectorState, targetState);
        if (DebugStateTransit::GetTransitionMap().contains(transitionKey))
        {
            return true;
        }
        return false;
    }

    bool KrakenEffectorComponent::IsTransitionAcceptable(EffectorState targetState) const
    {
        if (m_effectorState != m_effectorTargetState)
        {
            AZ_Error(
                "KrakenEffectorComponent",
                false,
                "Unable to accept request: currently realizing %s",
                DebugStateTransit::StateTransitionString(m_effectorState, m_effectorTargetState).c_str());
            return false;
        }

        if (!IsTransitionValid(targetState))
        {
            AZ_Error(
                "KrakenEffectorComponent",
                false,
                "Invalid state transition %s",
                DebugStateTransit::StateTransitionString(m_effectorState, m_effectorTargetState).c_str());
            return false;
        }

        return true;
    }

    void KrakenEffectorComponent::BeginTransitionIfAcceptable(EffectorState targetState)
    {
        if (IsTransitionAcceptable(targetState))
        {
            m_currentStateTransitionTime = 0.0f;
            m_effectorTargetState = targetState;
        }
    }

    void KrakenEffectorComponent::PrepareForPicking()
    {
        AZ_TracePrintf("KrakenEffectorComponent", "PrepareForPicking\n");
        BeginTransitionIfAcceptable(EffectorState::PREPARED);
    }

    void KrakenEffectorComponent::PickApple(const PickAppleTask& appleTask)
    {
        AZ_TracePrintf("KrakenEffectorComponent", "PickApple\n");
        // TODO - handle appleTask
        m_currentTask = appleTask;
        BeginTransitionIfAcceptable(EffectorState::PICKING);
    }

    void KrakenEffectorComponent::FinishPicking()
    {
        AZ_TracePrintf("KrakenEffectorComponent", "FinishPicking\n");
        BeginTransitionIfAcceptable(EffectorState::IDLE);
    }

    PickingState KrakenEffectorComponent::GetEffectorState()
    {
        PickingState state;
        state.m_effectorState = m_effectorState;
        state.m_taskProgress = 0.0f; // TODO
        if (m_currentTask.IsValid())
        {
            state.m_currentTask = m_currentTask;
        }
        return state;
    }

    AZ::Obb KrakenEffectorComponent::GetEffectorReachArea()
    {
        AZ_TracePrintf("KrakenEffectorComponent", "GetEffectorReachArea\n");
        AZ::Obb reachArea;

        if (m_reachEntity.IsValid())
        {
            AZ::Transform targetTM = AZ::Transform::CreateIdentity();
            AZ::TransformBus::EventResult(targetTM, m_reachEntity, &AZ::TransformBus::Events::GetWorldTM);
            AZ::Vector3 dimensions = AZ::Vector3{ 0.f };
            LmbrCentral::BoxShapeComponentRequestsBus::EventResult(
                dimensions, m_reachEntity, &LmbrCentral::BoxShapeComponentRequests::GetBoxDimensions);
            if (!dimensions.IsZero())
            {
                AZ_Printf("KrakenEffectorComponent", "OurEffectorReachArea :");
                AZ_Printf(
                    "KrakenEffectorComponent", "  local dimensions : %f %f %f", dimensions.GetX(), dimensions.GetY(), dimensions.GetZ());
                AZ_Printf(
                    "KrakenEffectorComponent",
                    "  transform - rot  : %f %f %f %f",
                    targetTM.GetRotation().GetX(),
                    targetTM.GetRotation().GetY(),
                    targetTM.GetRotation().GetZ(),
                    targetTM.GetRotation().GetW());
                AZ_Printf(
                    "KrakenEffectorComponent",
                    "  transform - pos  : %f %f %f",
                    targetTM.GetTranslation().GetX(),
                    targetTM.GetTranslation().GetY(),
                    targetTM.GetTranslation().GetZ());

                reachArea.SetHalfLengths(dimensions / 2);
                reachArea.SetPosition(targetTM.GetTranslation());
                reachArea.SetRotation(targetTM.GetRotation());

                return reachArea;
            }
            AZ_Warning(
                "KrakenEffectorComponent", true, "Reach entity %s does not have BoxShapeComponent!", m_reachEntity.ToString().c_str());
        }
        AZ_Warning("KrakenEffectorComponent", true, "GetEffectorReachArea - returning invalid reach");
        reachArea.SetHalfLengths(AZ::Vector3{ 0, 0, 0 });
        reachArea.SetPosition(AZ::Vector3{ 0, 0, 0 }); /// TODO - get it from entity With box

        return reachArea;
    }

    void KrakenEffectorComponent::OnEffectorReadyForPicking()
    {
        ApplePickingNotificationBus::Broadcast(&ApplePickingNotifications::EffectorReadyForPicking);
    }

    void KrakenEffectorComponent::OnApplePicked()
    {
        if (!m_currentTask.IsValid())
        {
            AZ_Error("KrakenEffectorComponent", true, "No valid task for current picking!");
            return;
        }

        // TODO - also handle picking failed
        ApplePickingNotificationBus::Broadcast(&ApplePickingNotifications::ApplePicked);

        // Just entered retrieving state, transition to Prepared
        BeginTransitionIfAcceptable(EffectorState::PREPARED);
    }

    void KrakenEffectorComponent::OnAppleRetrieved()
    {
        ApplePickingNotificationBus::Broadcast(&ApplePickingNotifications::AppleRetrieved);
    }

    void KrakenEffectorComponent::OnTick(float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if (m_effectorState == m_effectorTargetState)
        { // //TODO - nothing to do in stub version
            return;
        }

        m_currentStateTransitionTime += deltaTime;
        if (m_currentStateTransitionTime < DebugStateTransit::GetStateTransitTime(m_effectorState, m_effectorTargetState))
        {
            return;
        }

        // State transition
        AZ_TracePrintf(
            "KrakenEffectorComponent", "%s", DebugStateTransit::StateTransitionString(m_effectorState, m_effectorTargetState).c_str());
        m_currentStateTransitionTime = 0.0f;

        // Update state
        auto previousState = m_effectorState;
        m_effectorState = m_effectorTargetState;

        if (previousState == EffectorState::IDLE && m_effectorState == EffectorState::PREPARED)
        {
            OnEffectorReadyForPicking();
        }

        if (previousState == EffectorState::PREPARED && m_effectorState == EffectorState::PICKING)
        {
            // start picking the apple
            BeginTransitionIfAcceptable(EffectorState::RETRIEVING);
        }

        if (previousState == EffectorState::PICKING && m_effectorState == EffectorState::RETRIEVING)
        {
            OnApplePicked();
        }

        if (previousState == EffectorState::RETRIEVING && m_effectorState == EffectorState::PREPARED)
        {
            OnAppleRetrieved();
        }
    }
} // namespace AppleKraken
