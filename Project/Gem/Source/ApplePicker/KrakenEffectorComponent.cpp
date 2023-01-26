/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "KrakenEffectorComponent.h"
#include "ApplePickingNotifications.h"
#include "Manipulator/ManipulatorRequestBus.h"
#include "PickingStructs.h"
#include "ROS2/VehicleDynamics/VehicleInputControlBus.h"
#include <AzCore/Component/Entity.h>
#include <AzCore/Component/ComponentApplicationBus.h>
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
        static const AZStd::unordered_map<EffectorState, const char*> kMapToString{ { EffectorState::INVALID, "INVALID" },
                                                                                { EffectorState::IDLE, "IDLE" },
                                                                                { EffectorState::PREPARED, "PREPARED" },
                                                                                { EffectorState::PICKING, "PICKING" },
                                                                                { EffectorState::PICKING_STABILIZE, "PICKING_STABILIZE" },
                                                                                { EffectorState::RETRIEVING_NOSE, "RETRIEVING_NOSE" },
                                                                                { EffectorState::RETRIEVING, "RETRIEVING" },
                                                                                { EffectorState::RETRIEVING_STABILIZE,
                                                                                  "RETRIEVING_STABILIZE" },
                                                                                { EffectorState::RETRIEVING_FAILED,
                                                                                            "RETRIEVING_FAILED" }  };

        // TODO - this is a debug space for a stub implementation. Proper: a state transition machine with lambdas.
        AZStd::string StateTransitionString(EffectorState current, EffectorState next)
        {
            return AZStd::string::format("state transition %s -> %s\n", kMapToString.at(current), kMapToString.at(next));
        }
    } // namespace DebugStateTransit

    KrakenEffectorComponent::KrakenEffectorComponent()
    {
        InitializeStateProperties();
    }

    void KrakenEffectorComponent::Activate()
    {
        ApplePickingRequestBus::Handler::BusConnect(GetEntityId());
        AZ::TickBus::Handler::BusConnect();
        ImGui::ImGuiUpdateListenerBus::Handler::BusConnect();

        EBUS_EVENT_ID_RESULT(m_appleProbe, m_manipulatorEntity, ManipulatorRequestBus, GetEffectorEntity);
        EBUS_EVENT_ID_RESULT(m_restEntityId, m_manipulatorEntity, ManipulatorRequestBus, GetRestEntity);

        m_onTriggerHandleBeginHandler = AzPhysics::SimulatedBodyEvents::OnTriggerEnter::Handler(
            [&]([[maybe_unused]] AzPhysics::SimulatedBodyHandle bodyHandle, [[maybe_unused]] const AzPhysics::TriggerEvent& event)
            {
                const AZ::EntityId& e1 = event.m_otherBody->GetEntityId();
                const AZ::EntityId& e2 = event.m_triggerBody->GetEntityId();
                [[maybe_unused]] const AZ::EntityId& collideToEntityId = m_appleProbe == e1 ? e2 : e1;
                //                AZStd::string entity_name;
                //                AZ::ComponentApplicationBus::BroadcastResult(entity_name,
                //                &AZ::ComponentApplicationRequests::GetEntityName, collideToEntityId);
                //                AZ_Printf("m_onTriggerHandleBeginHandler", "Collission %s\n", entity_name.c_str());
                // AzPhysics::SimulatedBody* collideToEntityId = this->GetEntityId() == e1 ?  event.m_triggerBody : event.m_otherBody;}
                if (m_currentTask.m_appleEntityId == collideToEntityId)
                {
                    AZ_TracePrintf("m_onTriggerHandleBeginHandler", " %s : m_onTriggerHandle to Apple!====================",
                                   GetEntity()->GetName().c_str());
                    ApplePickingNotificationBus::Event(this->GetEntityId(),&ApplePickingNotifications::ApplePicked);
                    if (m_effectorState == EffectorState::PICKING)
                    {
                        // start picking the apple
                        BeginTransitionIfAcceptable(EffectorState::PICKING_STABILIZE);
                    }
                }
                if (m_restEntityId == collideToEntityId)
                {
                    AZ_TracePrintf("m_onTriggerHandleBeginHandler", "%s : m_onTriggerHandle to Rest!====================",
                                   GetEntity()->GetName().c_str());
                    if (m_effectorState == EffectorState::RETRIEVING || m_effectorState == EffectorState::RETRIEVING_NOSE )
                    {
                        // start picking the apple
                        BeginTransitionIfAcceptable(EffectorState::RETRIEVING_STABILIZE);
                    }
                }
            });
    }

    void KrakenEffectorComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        ApplePickingRequestBus::Handler::BusDisconnect();
        ImGui::ImGuiUpdateListenerBus::Handler::BusDisconnect();
    }

    void KrakenEffectorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<KrakenEffectorComponent, AZ::Component>()
                ->Version(4)
                ->Field("ReachEntity", &KrakenEffectorComponent::m_reachEntity)
                ->Field("ManipulatorEntity", &KrakenEffectorComponent::m_manipulatorEntity)
                ->Field("RootManipulatorFreeze", &KrakenEffectorComponent::m_rootEntityToFreeze)
                ->Field("BaseLinkToKinematic", &KrakenEffectorComponent::m_baseLinkToKinematic)
                ->Field("PickStabilizeTime", &KrakenEffectorComponent::m_stabilize_time)
                ->Field("MaxPickingTime", &KrakenEffectorComponent::m_maxPickingTime);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<KrakenEffectorComponent>("Kraken Effector", "Manipulator component for picking apples")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "AppleKraken")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::EntityId,
                        &KrakenEffectorComponent::m_reachEntity,
                        "Kraken Reach entity",
                        "Kraken entity with box shape to set reach area")
                    ->DataElement(
                        AZ::Edit::UIHandlers::EntityId,
                        &KrakenEffectorComponent::m_manipulatorEntity,
                        "Entity with manipulator",
                        "The entity that has a component handling events from ManipulatorRequestBus")
                    ->DataElement(
                        AZ::Edit::UIHandlers::EntityId,
                        &KrakenEffectorComponent::m_rootEntityToFreeze,
                        "RootManipulatorFreeze",
                        "RootManipulatorFreeze to freeze during robot movement")
                    ->DataElement(
                        AZ::Edit::UIHandlers::EntityId,
                        &KrakenEffectorComponent::m_baseLinkToKinematic,
                        "BaseLinkToKinematic",
                        "BaseLinkToKinematic during manipulator movement")
                    ->DataElement(
                        AZ::Edit::UIHandlers::EntityId,
                        &KrakenEffectorComponent::m_stabilize_time,
                        "PickStabilizeTime",
                        "PickStabilizeTime")
                    ->DataElement(
                        AZ::Edit::UIHandlers::EntityId, &KrakenEffectorComponent::m_maxPickingTime, "MaxPickingTime", "MaxPickingTime")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true);
            }
        }
    }

    void KrakenEffectorComponent::PrepareForPicking()
    {
        AZ_TracePrintf("KrakenEffectorComponent", "%s: PrepareForPicking\n", GetEntity()->GetName().c_str());
        BeginTransitionIfAcceptable(EffectorState::PREPARED);
    }

    void KrakenEffectorComponent::PickApple(const PickAppleTask& appleTask)
    {
        AZ_TracePrintf("KrakenEffectorComponent", "%s: PickApple\n", GetEntity()->GetName().c_str());
        // TODO - handle appleTask
        m_currentTask = appleTask;
        ManipulatorRequestBus::Event(m_manipulatorEntity, &ManipulatorRequest::PickApple, appleTask.m_middle);
        BeginTransitionIfAcceptable(EffectorState::PICKING);
    }

    void KrakenEffectorComponent::FinishPicking()
    {
        AZ_TracePrintf("KrakenEffectorComponent", "%s : FinishPicking\n", GetEntity()->GetName().c_str());
        BeginTransitionIfAcceptable(EffectorState::IDLE);
    }

    PickingState KrakenEffectorComponent::GetEffectorState()
    {
        PickingState state;
        state.m_effectorState = m_effectorState;
        state.m_taskProgress = 0.0f; // TODO
        state.m_description = DebugStateTransit::kMapToString.at(m_effectorState);
        if (m_currentTask.IsValid())
        {
            state.m_currentTask = m_currentTask;
        }
        return state;
    }

    AZ::Obb KrakenEffectorComponent::GetEffectorReachArea()
    {
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

    void KrakenEffectorComponent::OnTick(float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        m_currentStateTransitionTime += deltaTime;
        GetCurrentStateAction()();

        if (m_effectorState == m_effectorTargetState)
        { // //TODO - nothing to do in stub version
            return;
        }

        // State transition
        AZ_TracePrintf(
            "KrakenEffectorComponent", "%s : %s", GetEntity()->GetName().c_str(), DebugStateTransit::StateTransitionString(m_effectorState, m_effectorTargetState).c_str());
        m_currentStateTransitionTime = 0.0f;

        // Update state
        auto transitionAction = GetCurrentTransitionAction();
        m_effectorState = m_effectorTargetState;

        transitionAction();

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
            if (locked) {
                AZ_Printf("KrakenEffectorComponent", "Locking : %s\n", GetEntity()->GetName().c_str());
            }
            else{
                AZ_Printf("KrakenEffectorComponent", "Unlocking : %s\n", GetEntity()->GetName().c_str());
            }
            for (auto& descadant : descendants)
            {
                using VehicleBus = ROS2::VehicleDynamics::VehicleInputControlRequestBus;
                if (locked)
                {
                    // Lock manipulator, make base_link not kinematic anymore
                    Physics::RigidBodyRequestBus::Event(descadant, &Physics::RigidBodyRequestBus::Events::DisablePhysics);
                    Physics::RigidBodyRequestBus::Event(m_baseLinkToKinematic, &Physics::RigidBodyRequestBus::Events::SetKinematic, false);
                    VehicleBus::Event(m_baseLinkToKinematic, &VehicleBus::Events::SetDisableVehicleDynamics, false);
                }
                else
                {
                    // loose manipulator, make base_link kinematic
                    Physics::RigidBodyRequestBus::Event(descadant, &Physics::RigidBodyRequestBus::Events::EnablePhysics);
                    Physics::RigidBodyRequestBus::Event(m_baseLinkToKinematic, &Physics::RigidBodyRequestBus::Events::SetKinematic, true);
                    VehicleBus::Event(m_baseLinkToKinematic, &VehicleBus::Events::SetDisableVehicleDynamics, true);
                }
            }
            is_manipulator_locked = locked;
        }
    }

    bool KrakenEffectorComponent::IsTransitionValid(EffectorState targetState) const
    {
        AZ_Assert(m_effectorState != EffectorState::INVALID, "Effector is in an invalid state! Unable to access transition properties.");
        return m_stateProperties.m_allowedTransitions.contains(AZStd::make_pair(m_effectorState, targetState));
    }

    bool KrakenEffectorComponent::IsTransitionAcceptable(EffectorState targetState) const
    {
        if (m_effectorState == EffectorState::PICKING && m_effectorState == EffectorState::PICKING)
        {
            // allow this non-existing state transition without error
            return true;
        }

        if (m_effectorState != m_effectorTargetState)
        {
            AZ_Error(
                "KrakenEffectorComponent",
                false,
                "%s: Unable to accept request: currently realizing %s", GetEntity()->GetName().c_str(),
                DebugStateTransit::StateTransitionString(m_effectorState, m_effectorTargetState).c_str());
            return false;
        }

        if (!IsTransitionValid(targetState))
        {
            AZ_Error(
                "KrakenEffectorComponent",
                false,
                "%s: Invalid state transition %s",GetEntity()->GetName().c_str(),
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

    void KrakenEffectorComponent::InitializeStateProperties()
    {
        m_stateProperties.m_stateActions = {
            { EffectorState::IDLE,
              [this]()
              {
                  LockManipulator(true);
              } },
            { EffectorState::PREPARED,
              []()
              {
              } },
            { EffectorState::PICKING,
              [this]()
              {
                  if (m_currentStateTransitionTime > m_maxPickingTime)
                  {
                      AZ_Printf("m_onTriggerHandleBeginHandler", "%s : Failed to retrieve apple--------------------\n", GetEntity()->GetName().c_str());
                      BeginTransitionIfAcceptable(EffectorState::RETRIEVING_FAILED);
                  }
              } },
            { EffectorState::PICKING_STABILIZE,
              [this]()
              {
                  if (m_currentStateTransitionTime > m_stabilize_time)
                  {
                      BeginTransitionIfAcceptable(EffectorState::RETRIEVING_NOSE);
                  }
              } },
            { EffectorState::RETRIEVING_NOSE,
              [this]()
              {
                bool result;
                EBUS_EVENT_ID_RESULT(result, m_manipulatorEntity, ManipulatorRequestBus, IsNoseRetreived);

                if (result)
                {
                    BeginTransitionIfAcceptable(EffectorState::RETRIEVING);
                }
              } },
            { EffectorState::RETRIEVING_FAILED,
              [this]()
              {
                bool result;
                EBUS_EVENT_ID_RESULT(result, m_manipulatorEntity, ManipulatorRequestBus, IsNoseRetreived);

                if (result)
                {
                      BeginTransitionIfAcceptable(EffectorState::PREPARED);
                }
              } },
            {EffectorState::RETRIEVING,
              [this]() {
                  // Continue if manipulator retraction was blocked
                  if (m_currentStateTransitionTime > m_maxRetrieveTime) {
                      BeginTransitionIfAcceptable(EffectorState::RETRIEVING_STABILIZE);
                  }
              } },
            { EffectorState::RETRIEVING_STABILIZE,
              [this]()
              {
                  if (m_currentStateTransitionTime > m_stabilize_time)
                  {
                      BeginTransitionIfAcceptable(EffectorState::PREPARED);
                  }
              } }

        };

        m_stateProperties.m_allowedTransitions = {
            {
                { EffectorState::IDLE, EffectorState::PREPARED },
                [this]()
                {
                    LockManipulator(false);
                    ApplePickingNotificationBus::Event(GetEntityId(),&ApplePickingNotifications::EffectorReadyForPicking);
                },
            },
            {
                { EffectorState::PREPARED, EffectorState::PICKING },
                []()
                {
                },
            },
            {
                { EffectorState::PICKING, EffectorState::PICKING_STABILIZE },
                []()
                {
                },
            },
            {
                { EffectorState::PICKING_STABILIZE, EffectorState::RETRIEVING_NOSE },
                [this]()
                {
                    if (!m_currentTask.IsValid())
                    {
                        AZ_Error("KrakenEffectorComponent", true, "%s : No valid task for current picking!",GetEntity()->GetName().c_str());
                        return;
                    }
                    ManipulatorRequestBus::Event(m_manipulatorEntity, &ManipulatorRequest::RetrieveNose);
                },
            },
            {
                { EffectorState::RETRIEVING_NOSE, EffectorState::RETRIEVING },
                [this]()
                {
                    ManipulatorRequestBus::Event(m_manipulatorEntity, &ManipulatorRequest::Retrieve);
                },
            },
            { // on skip
                    { EffectorState::RETRIEVING_NOSE, EffectorState::RETRIEVING_STABILIZE },
                    [this]()
                    {
                        ManipulatorRequestBus::Event(m_manipulatorEntity, &ManipulatorRequest::Retrieve);
                    },
            },
            {
                { EffectorState::RETRIEVING, EffectorState::RETRIEVING_STABILIZE },
                []()
                {
                },
            },
            {
                { EffectorState::RETRIEVING_FAILED, EffectorState::PREPARED },
                [this]()
                {
                    ApplePickingNotificationBus::Event(this->GetEntityId(),&ApplePickingNotifications::PickingFailed, "Timeout");
                },
            },

            {
                { EffectorState::RETRIEVING_STABILIZE, EffectorState::PREPARED },
                [this]()
                {
                    ApplePickingNotificationBus::Event(GetEntityId(),&ApplePickingNotifications::AppleRetrieved);
                },
            },

            {
                { EffectorState::PREPARED, EffectorState::IDLE },
                []()
                {
                },
            },
            {
                { EffectorState::PICKING, EffectorState::IDLE },
                [this]()
                {
                    // apple picking was finished with timeout
                    ManipulatorRequestBus::Event(m_manipulatorEntity, &ManipulatorRequest::Retrieve);
                },
            },
            {
                { EffectorState::PICKING, EffectorState::RETRIEVING_FAILED },
                [this]()
                {
                    ManipulatorRequestBus::Event(m_manipulatorEntity, &ManipulatorRequest::RetrieveNose);
                    ManipulatorRequestBus::Event(m_manipulatorEntity, &ManipulatorRequest::ResetApple);
                },
            },
        };
    }

    const AZStd::function<void()>& KrakenEffectorComponent::GetCurrentStateAction() const
    {
        AZ_Assert(m_effectorState != EffectorState::INVALID, "%s : Effector is in an invalid state! Unable to access state properties.",GetEntity()->GetName().c_str());
        return m_stateProperties.m_stateActions.at(m_effectorState);
    }

    const AZStd::function<void()>& KrakenEffectorComponent::GetCurrentTransitionAction() const
    {
        return m_stateProperties.m_allowedTransitions.at(AZStd::make_pair(m_effectorState, m_effectorTargetState));
    }

    void KrakenEffectorComponent::OnImGuiUpdate(){

        AZStd::string window_name = AZStd::string::format("ManipulatorController%s", GetEntityId().ToString().c_str());
        ImGui::Begin(window_name.c_str());
        const auto & state_name = DebugStateTransit::kMapToString.at(m_effectorState);
        ImGui::Text("m_effectorState : %s",state_name);
        ImGui::BeginGroup();
        ImGui::Text("m_currentTask:");
        ImGui::Text("m_appleEntityId : %s",m_currentTask.m_appleEntityId.ToString().c_str());
        ImGui::Text("m_middle : %.1f %.1f %.1f",m_currentTask.m_middle.GetX(),m_currentTask.m_middle.GetY(),m_currentTask.m_middle.GetZ());
        ImGui::EndGroup();
        if (ImGui::CollapsingHeader("KrakenTestApplePicking") && m_reachEntity.IsValid()) {
            AZ::Obb r = KrakenEffectorComponent::GetEffectorReachArea();

            ImGui::SliderFloat("Horizontal", &m_debugApple[0], -r.GetHalfLengthX(), r.GetHalfLengthX());
            ImGui::SliderFloat("Vertical", &m_debugApple[2], -r.GetHalfLengthZ(), r.GetHalfLengthZ());
            ImGui::SliderFloat("Nose", &m_debugApple[1], -r.GetHalfLengthY(), r.GetHalfLengthY());

            if (ImGui::Button("Send 'PickApple'")) {
                AZ::Transform targetTM = AZ::Transform::CreateIdentity();
                AZ::TransformBus::EventResult(targetTM, m_reachEntity, &AZ::TransformBus::Events::GetWorldTM);
                PickAppleTask appleTask;
                appleTask.m_middle = targetTM.TransformPoint(AZ::Vector3::CreateFromFloat3(m_debugApple.data()));
                PickApple(appleTask);
            }
            ImGui::SameLine();
            if (ImGui::Button("Send 'PrepareForPicking'")) {
                PrepareForPicking();
            }
            ImGui::SameLine();
            if (ImGui::Button("Send 'FinishPicking'")) {
                FinishPicking();
            }
        }
        ImGui::End();
    };
} // namespace AppleKraken
