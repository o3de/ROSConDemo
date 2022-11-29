/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "ManipulatorRequestBus.h"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzFramework/AzFrameworkModule.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>
#include <ROS2/Manipulator/MotorizedJointComponent.h>
#include <ImGuiBus.h>
#include <ImGui/ImGuiPass.h>

namespace AppleKraken
{
    //! Component responsible for storing counters of apples gathered by Kraken.
    class ManipulatorController
        : public AZ::Component
        , public ManipulatorRequestBus ::Handler
        , public ImGui::ImGuiUpdateListenerBus::Handler
        , public AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(ManipulatorController, "{1C292F1D-F050-42EB-81C1-F6F83C4929F4}", AZ::Component, ManipulatorRequestBus::Handler);

        // AZ::Component interface implementation.
        ManipulatorController() = default;
        ~ManipulatorController() = default;
        void Activate() override;
        void Deactivate() override;
        static void Reflect(AZ::ReflectContext* context);

    private:
        bool initialized{ false };
        ROS2::MotorizedJointComponent* getMotorizedJointComponent(const AZ::EntityId& entityWithMotJoint);
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        void OnImGuiUpdate() override;
        void PickApple(const AZ::Vector3 position) override;
        AZ::Vector3 GetPosition() override;
        void Retrieve() override;
        void ResetApple() override;
        void RetrieveNose() override;
        int GetStatus() override;
        bool IsNoseRetreived() override;
        AZ::EntityId GetEffectorEntity() override;
        AZ::EntityId GetRestEntity() override;

        void ResetTimer();

        AZ::Vector3 m_desiredPosition{0, 0, 0 };
        AZStd::optional<AZ::Vector3> m_desiredApple;
        bool m_noseRetrieveRequest{false };
        bool m_noseRetrievingSuccess{false};

        AZ::Vector3 m_vectorX{1, 0, 0 };
        AZ::Vector3 m_vectorY{0, 1, 0 };
        AZ::Vector3 m_vectorZ{0, 0, 1 };

        AZ::EntityId m_entityX;
        AZ::EntityId m_entityY;
        AZ::EntityId m_entityZ;
        AZ::EntityId m_effector;
        AZ::EntityId m_restEntity;

        AZ::Transform m_transform_base_link_to_effector;
        float m_setPointX{ 0 };
        float m_setPointY{ 0 };
        float m_setPointZ{ 0 };

        float max_errorXZ{ 0.05 };
        float max_errorY{ 0.05 };
        float m_timeSetpointReach{ 0.2 };

        float m_time_XZ_ok { 0.0 };
        float m_time_Y_ok { 0.0 };
        bool m_imguiManualControl{false};

    };
} // namespace AppleKraken
