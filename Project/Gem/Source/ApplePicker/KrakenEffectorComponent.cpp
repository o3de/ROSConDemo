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
                                                                                { EffectorState::RETRIEVING, "RETRIEVING" },
                                                                                { EffectorState::RETRIEVING_NOSE, "RETRIEVING_NOSE" },
                                                                                { EffectorState::RETRIEVING_STABILIZE,
                                                                                  "RETRIEVING_STABILIZE" } };

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
    }

    void KrakenEffectorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<KrakenEffectorComponent, AZ::Component>()
                ->Version(4)
                ->Field("ReachEntity", &KrakenEffectorComponent::m_reachEntity)
                ->Field("ManipulatorEntity", &KrakenEffectorComponent::m_manipulatorEntity)
                ->Field("AppleProbe", &KrakenEffectorComponent::m_appleProbe)
                ->Field("RootManipulatorFreeze", &KrakenEffectorComponent::m_rootEntityToFreeze)
                ->Field("BaseLinkToKinematic", &KrakenEffectorComponent::m_baseLinkToKinematic)
                ->Field("RestEntity", &KrakenEffectorComponent::m_restEntityId)
                ->Field("RetrieveNoseTime", &KrakenEffectorComponent::m_retrieve_nose_time)
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
                        AZ::Edit::UIHandlers::EntityId, &KrakenEffectorComponent::m_appleProbe, "Entity to probe apples", "Sucking collider")
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
                        &KrakenEffectorComponent::m_restEntityId,
                        "ManipulatorRestPoint",
                        "ManipulatorRestPoint")
                    ->DataElement(
                        AZ::Edit::UIHandlers::EntityId,
                        &KrakenEffectorComponent::m_retrieve_nose_time,
                        "Nose Retrieve Time",
                        "Nose Retrieve Time")
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
        ManipulatorRequestHandler::Reflect(context);
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
        if (m_currentTask.IsValid())
        {
            state.m_currentTask = m_currentTask;
        }
        return state;
    }

    AZ::Obb KrakenEffectorComponent::GetEffectorReachArea()
    {
        AZ_TracePrintf("KrakenEffectorComponent", "%s: GetEffectorReachArea\n", GetEntity()->GetName().c_str());
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
                if (locked)
                {
                    // Lock manipulator, make base_link not kinematic anymore
                    Physics::RigidBodyRequestBus::Event(descadant, &Physics::RigidBodyRequestBus::Events::DisablePhysics);
                    Physics::RigidBodyRequestBus::Event(m_baseLinkToKinematic, &Physics::RigidBodyRequestBus::Events::SetKinematic, false);
                }
                else
                {
                    // loose manipulator, make base_link kinematic
                    Physics::RigidBodyRequestBus::Event(descadant, &Physics::RigidBodyRequestBus::Events::EnablePhysics);
                    Physics::RigidBodyRequestBus::Event(m_baseLinkToKinematic, &Physics::RigidBodyRequestBus::Events::SetKinematic, true);
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
                      ApplePickingNotificationBus::Event(this->GetEntityId(),&ApplePickingNotifications::PickingFailed, "Timeout");
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
                  if (m_currentStateTransitionTime > m_retrieve_nose_time)
                  {
                      BeginTransitionIfAcceptable(EffectorState::RETRIEVING);
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
} // namespace AppleKraken
