/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "KrakenEffectorComponent.h"
#include "ApplePickingNotifications.h"
#include "ManipulatorRequestBus.h"
#include "PickingStructs.h"
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Physics/Common/PhysicsSimulatedBody.h>
#include <AzFramework/Physics/RigidBodyBus.h>
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
                { std::make_pair(EffectorState::PICKING, EffectorState::RETRIEVING), 10.0f },
                { std::make_pair(EffectorState::RETRIEVING, EffectorState::PREPARED), 2.f },
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

        m_onTriggerHandleBeginHandler = AzPhysics::SimulatedBodyEvents::OnTriggerEnter::Handler(
            [&]([[maybe_unused]] AzPhysics::SimulatedBodyHandle bodyHandle, [[maybe_unused]] const AzPhysics::TriggerEvent& event)
            {
                const AZ::EntityId& e1 = event.m_otherBody->GetEntityId();
                const AZ::EntityId& e2 = event.m_triggerBody->GetEntityId();
                [[maybe_unused]] const AZ::EntityId& collideToEntityId = m_appleProbe == e1 ? e2 : e1;
                // AzPhysics::SimulatedBody* collideToEntityId = this->GetEntityId() == e1 ?  event.m_triggerBody : event.m_otherBody;}
                if (m_currentTask.m_appleEntityId == collideToEntityId)
                {
                    AZ_Printf("m_onTriggerHandleBeginHandler", "=================m_onTriggerHandle to Apple!====================");
                    ApplePickingNotificationBus::Broadcast(&ApplePickingNotifications::ApplePicked);
                    if (m_effectorState == EffectorState::PICKING)
                    {
                        // start picking the apple
                        BeginTransitionIfAcceptable(EffectorState::RETRIEVING);
                    }
                }
            });
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
            serialize->Class<KrakenEffectorComponent, AZ::Component>()
                ->Version(3)
                ->Field("ReachEntity", &KrakenEffectorComponent::m_reachEntity)
                ->Field("ManipulatorEntity", &KrakenEffectorComponent::m_manipulatorEntity)
                ->Field("AppleProbe", &KrakenEffectorComponent::m_appleProbe)
                ->Field("RootManipulatorFreeze", &KrakenEffectorComponent::m_rootEntityToFreeze)
                ->Field("BaseLinkToKinematic", &KrakenEffectorComponent::m_baseLinkToKinematic);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<KrakenEffectorComponent>("Kraken Effector", "Manipulator component for picking apples")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "AppleKraken")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &KrakenEffectorComponent::m_reachEntity,
                        "Kraken Reach entity",
                        "Kraken entity with box shape to set reach area")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &KrakenEffectorComponent::m_manipulatorEntity,
                        "Entity with manipulator",
                        "The entity that has a component handling events from ManipulatorRequestBus")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &KrakenEffectorComponent::m_appleProbe, "Entity to probe apples", "Sucking collider")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &KrakenEffectorComponent::m_rootEntityToFreeze,
                        "RootManipulatorFreeze",
                        "RootManipulatorFreeze to freeze during robot movement")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &KrakenEffectorComponent::m_baseLinkToKinematic,
                        "BaseLinkToKinematic",
                        "BaseLinkToKinematic during manipulator movement")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true);
            }
        }
        ManipulatorRequestHandler::Reflect(context);
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
        ManipulatorRequestBus::Event(m_manipulatorEntity, &ManipulatorRequest::PickApple, appleTask.m_middle);
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
        LockManipulator(false);
        ApplePickingNotificationBus::Broadcast(&ApplePickingNotifications::EffectorReadyForPicking);
    }

    void KrakenEffectorComponent::OnApplePicked()
    {
        if (!m_currentTask.IsValid())
        {
            AZ_Error("KrakenEffectorComponent", true, "No valid task for current picking!");
            return;
        }
        ManipulatorRequestBus::Event(m_manipulatorEntity, &ManipulatorRequest::Retrieve);
    }

    void KrakenEffectorComponent::OnAppleRetrieved()
    {
        ApplePickingNotificationBus::Broadcast(&ApplePickingNotifications::AppleRetrieved);
    }

    void KrakenEffectorComponent::OnTick(float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        m_currentStateTransitionTime += deltaTime;
        if (m_effectorState == EffectorState::PICKING && m_currentStateTransitionTime > 5)
        {
            AZ_Printf("m_onTriggerHandleBeginHandler", "---------------Failed to retrieve apple--------------------\n");
            ApplePickingNotificationBus::Broadcast(&ApplePickingNotifications::PickingFailed, "Timeout");
            BeginTransitionIfAcceptable(EffectorState::RETRIEVING);
        }

        if (m_effectorState == EffectorState::RETRIEVING)
        {
            int status = -1;
            ManipulatorRequestBus::EventResult(status, m_manipulatorEntity, &ManipulatorRequest::GetStatus);
            if (status == 10)
            {
                BeginTransitionIfAcceptable(EffectorState::PREPARED);
            }
        }
        if (m_effectorState == EffectorState::IDLE)
        {
            LockManipulator(true);
        }

        if (m_effectorState == m_effectorTargetState)
        { // //TODO - nothing to do in stub version
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

        if (previousState == EffectorState::PICKING && m_effectorState == EffectorState::RETRIEVING)
        {
            OnApplePicked();
        }

        if (previousState == EffectorState::RETRIEVING && m_effectorState == EffectorState::PREPARED)
        {
            OnAppleRetrieved();
        }
        if (!m_registeredCallback)
        {
            auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
            auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
            auto [physicScene, physicBody] = physicsSystem->FindAttachedBodyHandleFromEntityId(m_appleProbe);
            if (physicBody != AzPhysics::InvalidSimulatedBodyHandle && physicScene != AzPhysics::InvalidSceneHandle)
            {
                AzPhysics::SimulatedBody* simulated_body = sceneInterface->GetSimulatedBodyFromHandle(physicScene, physicBody);
                simulated_body->RegisterOnTriggerEnterHandler(m_onTriggerHandleBeginHandler);
                m_registeredCallback = true;
            }
        }
    }
    void KrakenEffectorComponent::LockManipulator(bool locked)
    {
        AZStd::vector<AZ::EntityId> descendants;
        AZ::TransformBus::EventResult(descendants, m_rootEntityToFreeze, &AZ::TransformBus::Events::GetAllDescendants);
        descendants.push_back(m_rootEntityToFreeze);
        if (is_manipulator_locked != locked)
        {
            for (auto& descadant : descendants)
            {
                if (locked)
                {
                    // Lock manipulator, make base_link not kinematic anymore
                    AZ_Printf("KrakenEffectorComponent", "Locking : %s\n", descadant.ToString().c_str());
                    Physics::RigidBodyRequestBus::Event(descadant, &Physics::RigidBodyRequestBus::Events::DisablePhysics);
                    Physics::RigidBodyRequestBus::Event(m_manipulatorEntity, &Physics::RigidBodyRequestBus::Events::SetKinematic, false);
                }
                else
                {
                    // loose manipulator, make base_link kinematic
                    Physics::RigidBodyRequestBus::Event(descadant, &Physics::RigidBodyRequestBus::Events::EnablePhysics);
                    Physics::RigidBodyRequestBus::Event(m_manipulatorEntity, &Physics::RigidBodyRequestBus::Events::SetKinematic, true);
                }
            }
            is_manipulator_locked = locked;
        }
    }
} // namespace AppleKraken
