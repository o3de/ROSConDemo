/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "../Manipulator/ManipulatorRequestBus.h"
#include "ApplePickingRequests.h"
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
#include <ImGuiBus.h>
#include <ImGui/ImGuiPass.h>

namespace AppleKraken
{
    //! Component for apple picking effector (manipulator)
    class KrakenEffectorComponent
        : public AZ::Component
        , protected ApplePickingRequestBus::Handler
        , protected AZ::TickBus::Handler
        , protected  ImGui::ImGuiUpdateListenerBus::Handler
    {
    public:
        AZ_COMPONENT(KrakenEffectorComponent, "{9206FC30-DF56-4246-8247-5D6B31603B53}");
        KrakenEffectorComponent();
        static void Reflect(AZ::ReflectContext* context);

    protected:
        void Activate() override;
        void Deactivate() override;

        void PrepareForPicking() override;
        void PickApple(const PickAppleTask& appleTask) override;
        void FinishPicking() override;
        PickingState GetEffectorState() override;
        AZ::Obb GetEffectorReachArea() override;

        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        void OnImGuiUpdate() override;

    private:
        void LockManipulator(bool locked);

        bool IsTransitionValid(EffectorState targetState) const;
        bool IsTransitionAcceptable(EffectorState targetState) const;
        void BeginTransitionIfAcceptable(EffectorState targetState);

        struct EffectorStateProperties
        {
        public:
            AZStd::unordered_map<EffectorState, AZStd::function<void()>> m_stateActions;
            AZStd::unordered_map<StateTransition, AZStd::function<void()>, TransitionHash> m_allowedTransitions;
        };

        void InitializeStateProperties();
        const AZStd::function<void()>& GetCurrentTransitionAction() const;
        const AZStd::function<void()>& GetCurrentStateAction() const;

        PickAppleTask m_currentTask; //!> valid if RETRIEVING or PICKING
        float m_maxPickingTime{ 5.0f };
        float m_currentStateTransitionTime = 0.0f;
        float m_stabilize_time{ 0.5f };
        const float m_maxRetrieveTime{ 3.5f };

        EffectorState m_effectorState = EffectorState::IDLE;
        EffectorState m_effectorTargetState = EffectorState::IDLE;
        EffectorStateProperties m_stateProperties;
        AZ::EntityId m_reachEntity;
        AZ::EntityId m_manipulatorEntity;
        AZ::EntityId m_rootEntityToFreeze;
        AZ::EntityId m_appleProbe;
        AZ::EntityId m_baseLinkToKinematic;
        AZ::EntityId m_restEntityId;

        bool m_registeredCallback{ false };
        bool is_manipulator_locked = { false };
        AzPhysics::SimulatedBodyEvents::OnTriggerEnter::Handler m_onTriggerHandleBeginHandler;
    };
} // namespace AppleKraken
