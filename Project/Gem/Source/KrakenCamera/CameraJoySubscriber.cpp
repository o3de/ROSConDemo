/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "CameraJoySubscriber.h"
#include "FollowingCameraComponent.h"

#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
namespace AppleKraken
{

    float getFromButtons(const AZStd::array<int,2>& config , const sensor_msgs::msg::Joy& msg )
    {
        auto bc = msg.buttons.size();
        if (config[0]>0 && config[0]<bc && config[1]>0  && config[1]<bc) {
            if (msg.buttons[config[0]]){ return 1.0f;}
            if (msg.buttons[config[1]]){ return -1.0f;}
            return 0.0f;
        }
        return 0.f;
    }
    void CameraJoySubscriber::Activate()
    {
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        m_joySubscriber = ros2Node->create_subscription<sensor_msgs::msg::Joy>(m_joyTopic.c_str(),10,
       [this](const sensor_msgs::msg::Joy::ConstSharedPtr msg)
       {
           m_input.SetX(getFromButtons(m_buttonsX, *msg));
           m_input.SetY(getFromButtons(m_buttonsY, *msg));
           auto bc = msg->buttons.size();
           if(m_buttonsX[0]>=bc ||m_buttonsX[1]>=bc || m_buttonsY[0]>=bc ||m_buttonsY[1]>=bc ){
               AZ_Warning("CameraJoySubscriber", false, "Joystick has only %d buttons!", bc);
           }
       });
       AZ::TickBus::Handler::BusConnect();
    }

    void CameraJoySubscriber::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
    }

    void CameraJoySubscriber::Reflect(AZ::ReflectContext* context) {
        if (AZ::SerializeContext *serialize = azrtti_cast<AZ::SerializeContext *>(context)) {
            serialize->Class<CameraJoySubscriber, AZ::Component>()
                    ->Version(1)
                    ->Field("JoyTopic", &CameraJoySubscriber::m_joyTopic)
                    ->Field("Speed", &CameraJoySubscriber::m_speeds)

                    ->Field("buttonsX", &CameraJoySubscriber::m_buttonsX)
                    ->Field("buttonsY", &CameraJoySubscriber::m_buttonsY);



            if (AZ::EditContext *ec = serialize->GetEditContext()) {
                ec->Class<CameraJoySubscriber>("CameraJoySubscriber", "CameraJoySubscriber")
                        ->ClassElement(AZ::Edit::ClassElements::EditorData, "CameraJoySubscriber")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "AppleKraken")

                        ->DataElement(AZ::Edit::UIHandlers::Default, &CameraJoySubscriber::m_joyTopic, "JoyTopic",
                                      "JoyTopic")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &CameraJoySubscriber::m_speeds, "Speed",
                                      "Speed")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &CameraJoySubscriber::m_buttonsX, "m_axisCameraX",
                                      "Buttons to operate camera axis 1")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &CameraJoySubscriber::m_buttonsY, "m_axisCameraY",
                                      "Buttons axis to operate camera axis 2");

            }
        }
    }

    void CameraJoySubscriber::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time){
        AppleKraken::FollowingCameraComponent * component = GetEntity()->FindComponent<AppleKraken::FollowingCameraComponent>();
        if (component)
        {
            component->Rotate(m_speeds*m_input);
        }else{
            AZ_Warning("CameraJoySubscriber", false, "This entity does not have AppleKraken::FollowingCameraComponent please fix it");
        }
    }

} // namespace AppleKraken
