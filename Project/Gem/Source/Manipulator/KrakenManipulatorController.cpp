/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "KrakenManipulatorController.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzFramework/Components/TransformComponent.h>
#include <AzToolsFramework/UI/PropertyEditor/PropertyEditorAPI.h>

namespace AppleKraken
{
    ROS2::MotorizedJoint* ManipulatorController::getMotorizedJoint(const AZ::EntityId& entityWithMotJoint)
    {
        AZ_Assert(entityWithMotJoint.IsValid(), "Entity Invalid");
        AZ::Entity* e_ptr{ nullptr };
        AZ::ComponentApplicationBus::BroadcastResult(e_ptr, &AZ::ComponentApplicationRequests::FindEntity, entityWithMotJoint);
        AZ_Assert(e_ptr, "No such entity");
        ROS2::MotorizedJoint* component = e_ptr->FindComponent<ROS2::MotorizedJoint>();
        //        AZ_Assert(component, "No  ROS2::MotorizedJoint in %s", e_ptr->GetName().c_str());
        return component;
    }

    void ManipulatorController::Activate()
    {
        ManipulatorRequestBus::Handler::BusConnect(GetEntityId());
        AZ::TickBus::Handler::BusConnect();
        ImGui::ImGuiUpdateListenerBus::Handler::BusConnect();
        initialized = false;
    }

    void ManipulatorController::Deactivate()
    {
        ManipulatorRequestBus::Handler::BusDisconnect(GetEntityId());
        AZ::TickBus::Handler::BusDisconnect();
        ImGui::ImGuiUpdateListenerBus::Handler::BusDisconnect();
    }

    void ManipulatorController::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ManipulatorController, AZ::Component>()
                ->Version(2)
                ->Field("ManipulatorEntityX", &ManipulatorController::m_entityX)
                ->Field("ManipulatorEntityY", &ManipulatorController::m_entityY)
                ->Field("ManipulatorEntityZ", &ManipulatorController::m_entityZ)
                ->Field("ManipulatorVecX", &ManipulatorController::m_vectorX)
                ->Field("ManipulatorVecY", &ManipulatorController::m_vectorY)
                ->Field("ManipulatorVecZ", &ManipulatorController::m_vectorZ)
                ->Field("RestEntity", &ManipulatorController::m_restEntity)
                ->Field("m_effector", &ManipulatorController::m_effector)
                ->Field("max_errorXZ", &ManipulatorController::max_errorXZ)
                ->Field("max_errorY", &ManipulatorController::max_errorY);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ManipulatorController>("ManipulatorController", "ManipulatorController")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "ManipulatorController")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "AppleKraken")
                    ->DataElement(AZ::Edit::UIHandlers::EntityId, &ManipulatorController::m_entityX, "m_entityX", "m_entityX")
                    ->DataElement(AZ::Edit::UIHandlers::EntityId, &ManipulatorController::m_entityY, "m_entityY", "m_entityY")
                    ->DataElement(AZ::Edit::UIHandlers::EntityId, &ManipulatorController::m_entityZ, "m_entity_z1", "m_entityZ")
                    ->DataElement(AZ::Edit::UIHandlers::EntityId, &ManipulatorController::m_vectorX, "vx", "vx")
                    ->DataElement(AZ::Edit::UIHandlers::EntityId, &ManipulatorController::m_vectorY, "vy", "vy")
                    ->DataElement(AZ::Edit::UIHandlers::EntityId, &ManipulatorController::m_vectorZ, "vz", "vz")
                    ->DataElement(AZ::Edit::UIHandlers::EntityId, &ManipulatorController::m_effector, "Effector", "Effector")
                    ->DataElement(AZ::Edit::UIHandlers::EntityId, &ManipulatorController::m_restEntity, "Rest entity", "Rest Entity")
                    ->DataElement(AZ::Edit::UIHandlers::EntityId, &ManipulatorController::max_errorXZ, "max_errorXZ", "max error XZ to retract nose")
                    ->DataElement(AZ::Edit::UIHandlers::EntityId, &ManipulatorController::max_errorY, "max_errorY", "max error Y to retract nose");
            }
        }
    }

    void ManipulatorController::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if (!initialized)
        {
            // get offset to effector
            AZ::Transform transformBaseLink;
            AZ::TransformBus::EventResult(transformBaseLink, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);
            AZ::Transform transformEffector;
            AZ::TransformBus::EventResult(transformEffector, m_effector, &AZ::TransformBus::Events::GetWorldTM);
            m_transform_base_link_to_effector = transformBaseLink.GetInverse() * transformEffector;

            initialized = true;
            return;
        }

        if (m_desiredApple)
        {
            m_desiredPosition = *m_desiredApple;
        }
        else
        {
            AZ::TransformBus::EventResult(m_desiredPosition, m_restEntity, &AZ::TransformBus::Events::GetWorldTranslation);
        }
        // apple is given in the World's coordinate system
        AZ::Transform current_base_link;
        AZ::TransformBus::EventResult(current_base_link, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);

        AZ::Vector3 position_in_baselink_tf = current_base_link.GetInverse().TransformPoint(m_desiredPosition);
        AZ::Vector3 position_in_effector_tf = m_transform_base_link_to_effector.GetInverse().TransformPoint(position_in_baselink_tf);

        m_setPointX = position_in_effector_tf.Dot(m_vectorX);
        m_setPointZ = position_in_effector_tf.Dot(m_vectorZ);

        float error_x = std::numeric_limits<float>::max();
        float error_z = std::numeric_limits<float>::max();

        if (m_entityX.IsValid())
        {
            auto component_x = getMotorizedJoint(m_entityX);
            if (component_x)
            {
                component_x->SetSetpoint(m_setPointX);
                error_x = component_x->GetError();
            }
        }

        if (m_entityZ.IsValid())
        {
            auto component_z = getMotorizedJoint(m_entityZ);
            if (component_z)
            {
                component_z->SetSetpoint(m_setPointZ);
                error_z = component_z->GetError();
            }
        }
        // auto - disable nose retrieve only if we reached small error.
        if (m_noseRetrieved == true)
        {
            m_time_XZ_ok += deltaTime;
            if (m_time_XZ_ok > m_timeSetpointReach)
            {
                if (error_x < max_errorXZ  && error_x > -max_errorXZ && error_z < max_errorXZ && error_z > -max_errorXZ)
                {
                    AZ_Printf("ManipulatorController", "Nose is sliding out  \n");
                    m_noseRetrieved = false;
                    m_time_XZ_ok = 0;
                }
            }
        }
        m_setPointY = position_in_effector_tf.Dot(m_vectorY);
        if (m_entityY.IsValid())
        {
            auto component_y = getMotorizedJoint(m_entityY);
            if (component_y)
            {
                if (m_noseRetrieved)
                {
                    m_setPointY = 0;
                    m_time_Y_ok += deltaTime;
                    if (m_time_Y_ok > m_timeSetpointReach)
                    {

                        auto error_y = getMotorizedJoint(m_entityY)->GetError();
                        if (error_y < max_errorY && error_y > -max_errorY) 
                        {
                            m_noseRetrievingSuccess = true;
                            m_time_Y_ok = 0.0;
                        }
                    }
                }
                else
                {
                    m_noseRetrievingSuccess = false;
                }
                component_y->SetSetpoint(m_setPointY);
            }
        }

    }

    void ManipulatorController::PickApple(const AZ::Vector3 position)
    {
        m_noseRetrieved = true;
        m_time_XZ_ok = 0;
        AZ_Printf("ManipulatorController", "PickApple\n");
        ResetTimer();
        m_desiredApple = position;
    };

    AZ::Vector3 ManipulatorController::GetPosition()
    {
        auto currentPosition = AZ::Vector3(
            getMotorizedJoint(m_entityX)->GetCurrentPosition(),
            getMotorizedJoint(m_entityY)->GetCurrentPosition(),
            getMotorizedJoint(m_entityZ)->GetCurrentPosition()
            );
        return currentPosition;
    };

    void ManipulatorController::Retrieve()
    {
        AZ_Printf("ManipulatorController", "Retrieve\n");
        m_time_XZ_ok = std::numeric_limits<float>::lowest();
        m_noseRetrieved = true;
        m_desiredApple.reset();
    };
    void ManipulatorController::RetrieveNose()
    {
        AZ_Printf("ManipulatorController", "RetrieveNose\n");
        m_time_XZ_ok = std::numeric_limits<float>::lowest();
        m_noseRetrieved = true;
    }

    int ManipulatorController::GetStatus()
    {
        return 0;
    };

    void ManipulatorController::ResetTimer()
    {
        AZ_Printf("ManipulatorController", "Timer is RESET!\n");
        m_time_XZ_ok = 0;
    }

    bool ManipulatorController::IsNoseRetreived()
    {
        return m_noseRetrievingSuccess;
    }

    AZ::EntityId ManipulatorController::GetEffectorEntity()
    {
        return m_effector;
    }

    AZ::EntityId ManipulatorController::GetRestEntity()
    {
        return m_restEntity;
    }

    void ManipulatorController::OnImGuiUpdate()
    {

        AZStd::string window_name("ManipulatorController %s", GetEntityId().ToString().c_str()));
        ImGui::Begin(window_name.c_str());
        auto pos = GetPosition();
        if (m_desiredApple){
            ImGui::Text("Desired Apple : %.1f %.1f %.1f", m_desiredApple->GetX(),m_desiredApple->GetY(),m_desiredApple->GetZ());
        }else{
            ImGui::Text("No Desired Apple");
        }
        if (ImGui::CollapsingHeader("Gantry"))
        {
            ImGui::Text("Positions : %.1f %.1f", pos.GetX(), pos.GetZ() );
            ImGui::Text("SetPoint  : %.1f %.1f", m_setPointX,  m_setPointZ );
        }
        if (ImGui::CollapsingHeader("Nose"))
        {
            ImGui::Text("Positions : %.1f", pos.GetY());
            ImGui::Text("SetPoint  : %.1f", m_setPointY);
        }
        ImGui::End();

    }

} // namespace AppleKraken
