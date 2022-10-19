/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ManipulatorJoySubscriber.h"
#include "KrakenManipulatorController.h"

#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Names.h>

#include <LmbrCentral/Shape/BoxShapeComponentBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Math/MathUtils.h>
namespace AppleKraken
{

    void ManipulatorJoySubscriber::Activate()
    {
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        m_joySubscriber = ros2Node->create_subscription<sensor_msgs::msg::Joy>(m_joyTopic.c_str(),10,
       [this](const sensor_msgs::msg::Joy::ConstSharedPtr msg)
       {
            auto ac = msg->axes.size();
            if (m_axisManipulatorX < ac && m_axisManipulatorY<ac && m_axisManipulatorZ<ac) {
                m_joyInput = AZ::Vector3{msg->axes[m_axisManipulatorX], msg->axes[m_axisManipulatorY], msg->axes[m_axisManipulatorZ]};
                if (m_joyInput.GetLength() > 0.25){
                    m_deactivate = false;
                }
            }else{
                AZ_Warning("ManipulatorJoySubscriber", false, "Joystick has only %d axes!", ac);
            }
       });
       ManipulatorRequestBus::Handler::BusConnect(GetEntityId());
       AZ::TickBus::Handler::BusConnect();
    }

    void ManipulatorJoySubscriber::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
    }

    void ManipulatorJoySubscriber::Reflect(AZ::ReflectContext* context) {
        if (AZ::SerializeContext *serialize = azrtti_cast<AZ::SerializeContext *>(context)) {
            serialize->Class<ManipulatorJoySubscriber, AZ::Component>()
                    ->Version(1)
                    ->Field("JoyTopic", &ManipulatorJoySubscriber::m_joyTopic)
                    ->Field("Speed", &ManipulatorJoySubscriber::m_speeds)
                    ->Field("ReachEntity", &ManipulatorJoySubscriber::m_reachEntity)
                    ->Field("ManipualtorEntity", &ManipulatorJoySubscriber::m_manipulatorEntity)
                    ->Field("AxisManipulatorX", &ManipulatorJoySubscriber::m_axisManipulatorX)
                    ->Field("AxisManipulatorY", &ManipulatorJoySubscriber::m_axisManipulatorY)
                    ->Field("AxisManipulatorZ", &ManipulatorJoySubscriber::m_axisManipulatorZ)
                    ->Field("DebugEntity", &ManipulatorJoySubscriber::m_debug);


            if (AZ::EditContext *ec = serialize->GetEditContext()) {
                ec->Class<ManipulatorJoySubscriber>("ManipulatorJoySubscriber", "ManipulatorJoySubscriber")
                        ->ClassElement(AZ::Edit::ClassElements::EditorData, "ManipulatorJoySubscriber")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "AppleKraken")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &ManipulatorJoySubscriber::m_joyTopic, "JoyTopic",
                                      "JoyTopic")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &ManipulatorJoySubscriber::m_axisManipulatorX, "JoyAxis1",
                                      "Joy axis to operate manipulator axis 1")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &ManipulatorJoySubscriber::m_axisManipulatorY, "JoyAxisNose",
                                      "Joy axis to operate manipulator axis 2")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &ManipulatorJoySubscriber::m_axisManipulatorZ, "JoyAxis3",
                                      "Joy axis to operate manipulator axis 3")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &ManipulatorJoySubscriber::m_speeds, "Speed",
                                      "Speed")
                        ->DataElement(
                                AZ::Edit::UIHandlers::EntityId,
                                &ManipulatorJoySubscriber::m_reachEntity,
                                "Kraken Reach entity",
                                "Kraken entity with box shape to set reach area")
                        ->DataElement(
                                AZ::Edit::UIHandlers::EntityId,
                                &ManipulatorJoySubscriber::m_manipulatorEntity,
                                "Kraken Manipulator entity",
                                "Entity with kraken manipulator")
                        ->DataElement(
                                AZ::Edit::UIHandlers::EntityId,
                                &ManipulatorJoySubscriber::m_debug,
                                "DebugEntity",
                                "DebugEntity");

            }
        }
    }

    void ManipulatorJoySubscriber::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time){
        if (m_deactivate) {
            return;
            AZ::TransformBus::Event( m_debug, &AZ::TransformBus::Events::SetWorldTranslation,AZ::Vector3{0,0,-5.f});
        }
        // manipulator
        if (m_reachEntity.IsValid() && m_manipulatorEntity.IsValid()) {
            AZ::Transform targetTM = AZ::Transform::CreateIdentity();
            AZ::TransformBus::EventResult(targetTM, m_reachEntity, &AZ::TransformBus::Events::GetWorldTM);
            LmbrCentral::BoxShapeComponentRequestsBus::EventResult(
                    m_reachDimension, m_reachEntity, &LmbrCentral::BoxShapeComponentRequests::GetBoxDimensions);
            m_currentSetpoint+= m_speeds *m_joyInput;

            m_currentSetpoint.SetX(AZ::GetClamp(m_currentSetpoint.GetX(), -0.5f*m_reachDimension.GetX(),0.5f*m_reachDimension.GetX() ));
            m_currentSetpoint.SetY(AZ::GetClamp(m_currentSetpoint.GetY(), -0.5f*m_reachDimension.GetY(),0.5f*m_reachDimension.GetY() ));
            m_currentSetpoint.SetZ(AZ::GetClamp(m_currentSetpoint.GetZ(), -0.5f*m_reachDimension.GetZ(),0.5f*m_reachDimension.GetZ() ));

            AZ::Vector3 t = targetTM.GetTranslation() + targetTM.TransformVector(m_currentSetpoint);

            AZ::Entity* m_manipulatorEntity_ptr{ nullptr };
            AZ::ComponentApplicationBus::BroadcastResult(m_manipulatorEntity_ptr, &AZ::ComponentApplicationRequests::FindEntity, m_manipulatorEntity);
            AppleKraken::ManipulatorController* component = m_manipulatorEntity_ptr->FindComponent<AppleKraken::ManipulatorController>();
            if (component) {
                component->GoToPosition(t);
            }
            AZ::TransformBus::Event( m_debug, &AZ::TransformBus::Events::SetWorldTranslation,t);
        }

    }

    void ManipulatorJoySubscriber::PickApple(const AZ::Vector3 position){
        m_deactivate = true;
        AZ_Warning("ManipulatorJoySubscriber", false, "Deactivating manual operation of Kraken Manipulator");
    }
} // namespace AppleKraken
