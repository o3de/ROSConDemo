/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "ApplePickingRequests.h"
#include "ManipulatorRequestBus.h"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/EBus/EBus.h>

#include <AzFramework/Physics/Common/PhysicsSceneQueries.h>
#include <AzFramework/Physics/Common/PhysicsSimulatedBody.h>
#include <AzFramework/Physics/Configuration/RigidBodyConfiguration.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/Shape.h>
#include <AzFramework/Physics/SystemBus.h>

namespace AppleKraken
{
    //! Component for apple picking effector (manipulator)
    class KrakenEffectorComponent
        : public AZ::Component
        , public ApplePickingRequestBus::Handler
        , public AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(KrakenEffectorComponent, "{9206FC30-DF56-4246-8247-5D6B31603B53}");
        KrakenEffectorComponent();
        static void Reflect(AZ::ReflectContext* context);

    private:
        void Activate() override;
        void Deactivate() override;
        void InitializeTransitionActions();

        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        void PrepareForPicking() override;
        void PickApple(const PickAppleTask& appleTask) override;
        void FinishPicking() override;
        PickingState GetEffectorState() override;
        AZ::Obb GetEffectorReachArea() override;
        std::function<void()> GetTransitionAction(EffectorState currentState, EffectorState targetState);

        void BeginTransitionIfAcceptable(EffectorState targetState);
        bool IsTransitionAcceptable(EffectorState targetState) const;
        bool IsTransitionValid(EffectorState targetState) const;

        void LockManipulator(bool locked);

        PickAppleTask m_currentTask; //!> valid if RETRIEVING or PICKING
        EffectorState m_effectorState = EffectorState::IDLE;
        EffectorState m_effectorTargetState = EffectorState::IDLE;
        float m_currentStateTransitionTime = 0.0f;
        // <startState, targetState>, onTransitionFinished
        AZStd::unordered_map<StateTransition, std::function<void()>, TransitionHash> m_stateTransitionActions;
        AZ::EntityId m_reachEntity;
        AZ::EntityId m_manipulatorEntity;
        AZ::EntityId m_rootEntityToFreeze;
        AZ::EntityId m_appleProbe;
        AZ::EntityId m_baseLinkToKinematic;

        bool m_registeredCallback{ false };
        bool is_manipulator_locked = { false };
        AzPhysics::SimulatedBodyEvents::OnTriggerEnter::Handler m_onTriggerHandleBeginHandler;
    };
} // namespace AppleKraken
