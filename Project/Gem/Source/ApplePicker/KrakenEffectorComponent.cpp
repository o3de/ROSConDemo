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
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>

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
                { std::make_pair(EffectorState::PREPARED, EffectorState::PICKING), 0.5f },
                { std::make_pair(EffectorState::PICKING, EffectorState::RETRIEVING), 2.0f },
                { std::make_pair(EffectorState::RETRIEVING, EffectorState::PREPARED), 2.0f },
                { std::make_pair(EffectorState::PREPARED, EffectorState::IDLE), 0.1f },
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
            serialize->Class<KrakenEffectorComponent, AZ::Component>()->Version(1);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<KrakenEffectorComponent>("Kraken Effector", "Manipulator component for picking apples")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"));
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
        return state;
    }

    AZ::Obb KrakenEffectorComponent::GetEffectorReachArea()
    {
        AZ_TracePrintf("KrakenEffectorComponent", "GetEffectorReachArea\n");
        AZ::Obb reachArea;
        // TODO - get the actual area
        return reachArea;
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
        m_effectorState = m_effectorTargetState;

        if (m_effectorState == EffectorState::PICKING && m_effectorTargetState == EffectorState::RETRIEVING)
        {
            // TODO - also handle picking failed
            ApplePickingNotificationBus::Broadcast(&ApplePickingNotifications::ApplePicked);

            // Start retrieving right away
            m_effectorTargetState = EffectorState::RETRIEVING;
            return;
        }

        if (m_effectorState == EffectorState::RETRIEVING && m_effectorTargetState == EffectorState::PREPARED)
        {
            ApplePickingNotificationBus::Broadcast(&ApplePickingNotifications::AppleRetrieved);
            return;
        }
    }
} // namespace AppleKraken
