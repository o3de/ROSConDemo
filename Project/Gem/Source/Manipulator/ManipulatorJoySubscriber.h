#pragma once
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
#include <ROS2/Manipulator/MotorizedJoint.h>
#include <AzCore/Component/TickBus.h>
#include "ROS2/ROS2Bus.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <AzFramework/Physics/Common/PhysicsSceneQueries.h>
#include <AzFramework/Physics/Common/PhysicsSimulatedBody.h>
#include <AzFramework/Physics/Configuration/RigidBodyConfiguration.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/Shape.h>
#include <AzFramework/Physics/SystemBus.h>


namespace AppleKraken
{
    class ManipulatorJoySubscriber
            : public AZ::Component
            , public AZ::TickBus::Handler
            , public ManipulatorRequestHandler

    {
    public:
        AZ_COMPONENT(ManipulatorJoySubscriber, "{618515CE-F44A-407D-8109-ACB247964C6B}", AZ::Component, ManipulatorRequestBus::Handler);

        // AZ::Component interface implementation.
        ManipulatorJoySubscriber() = default;
        ~ManipulatorJoySubscriber() = default;
        void Activate() override;
        void Deactivate() override;
        static void Reflect(AZ::ReflectContext* context);

    private:
        void PickApple(const AZ::Vector3 position) override;
        void OnTick(float deltaTime, AZ::ScriptTimePoint time);

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr m_joySubscriber;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_triggerService;

        AZStd::string m_joyTopic{"/joy"};
        AZ::Vector3 m_speeds {0.02,-0.01,0.02};
        AZ::EntityId m_reachEntity;
        AZ::EntityId m_manipulatorEntity;
        AZ::EntityId m_debug;
        AZ::EntityId m_appleProbe;
        AZ::EntityId m_entityToFail;

        AZ::Vector3 m_currentSetpoint{0,0,0};
        AZ::Vector3 m_joyInput{0,0,0};
        AZ::Vector3 m_reachDimension{1,1,1};
        uint16_t  m_axisManipulatorX {2};
        uint16_t  m_axisManipulatorY {5};
        uint16_t  m_axisManipulatorZ {3};

        bool m_registeredCallback{ false };
        AzPhysics::SimulatedBodyEvents::OnTriggerEnter::Handler m_onTriggerHandleBeginHandlerApple;
        AzPhysics::SimulatedBodyEvents::OnTriggerEnter::Handler m_onTriggerHandleBeginHandlerFail;


        bool m_deactivate{true};


    };
} // namespace AppleKraken
