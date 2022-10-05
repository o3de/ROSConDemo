/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ApplePickerComponent.h"
#include "ApplePickingRequests.h"
#include "FruitStorage/FruitStorageBus.h"
#include <AtomLyIntegration/CommonFeatures/Mesh/MeshComponentBus.h>
#include <AtomLyIntegration/CommonFeatures/Mesh/MeshComponentConstants.h>
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/EBus/Event.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/Shape.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Names.h>

using namespace ROS2;

namespace AppleKraken
{
    namespace Internal
    {
        AZStd::string TaskString(const PickAppleTask& task)
        {
            if (!task.m_appleEntityId.IsValid())
            {
                return "|Task for an unspecified apple|";
            }

            return AZStd::string::format("|Task for entity id %s|", task.m_appleEntityId.ToString().c_str());
        }

        AZStd::string CurrentTaskString(const AZStd::queue<PickAppleTask>& taskQueue)
        {
            if (taskQueue.empty())
            {
                return "|No task, pick queue empty!|";
            }

            const auto& currentTask = taskQueue.front();
            return TaskString(currentTask);
        }
    } // namespace Internal

    bool ApplePickerComponent::IsBusy() const
    {
        if (!m_currentAppleTasks.empty())
        { // busy - still has tasks in queue
            return true;
        }

        // There are no apple tasks, but the effector might not be idle or prepared yet
        PickingState pickingState;
        ApplePickingRequestBus::EventResult(pickingState, m_effectorEntityId, &ApplePickingRequests::GetEffectorState);
        if (pickingState.m_effectorState != EffectorState::IDLE && pickingState.m_effectorState != EffectorState::PREPARED)
        { // Effector is not ready - still transitioning to a state. This should be a rare occurrence.
            AZ_Warning("ApplePicker", false, "Task queue empty but apple picker is busy since the effector is working");
            return true;
        }
        return false;
    }

    void ApplePickerComponent::ProcessTriggerServiceCall(const TriggerRequest req, TriggerResponse resp)
    {
        // TODO - also, perhaps add a check whether Kraken is in gathering position, immobile etc.
        if (IsBusy())
        {
            resp->success = false;
            resp->message = "Unable to accept request - apple picker is currently busy";
            return;
        }

        resp->success = true;
        resp->message = "Command accepted, picking operation started";
        StartAutomatedOperation();
        return;
    }

    void ApplePickerComponent::StartAutomatedOperation()
    {
        if (IsBusy())
        {
            AZ_Error("ApplePicker", false, "Tasks still in progress for current picking!");
            return;
        }

        // Get effector reach
        AZ::Obb effectorRangeGlobalBox;
        ApplePickingRequestBus::EventResult(effectorRangeGlobalBox, m_effectorEntityId, &ApplePickingRequests::GetEffectorReachArea);

        // Find out apples within the reach
        QueryEnvironmentForAllApplesInBox(effectorRangeGlobalBox);

        // Tell effector to prepare for picking
        ApplePickingRequestBus::Event(m_effectorEntityId, &ApplePickingRequests::PrepareForPicking);
    }

    void ApplePickerComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        // TODO handle timeouts and incoming commands
    }

    float ApplePickerComponent::ReportProgress()
    {
        // TODO (minor) - take into consideration current task progress (effector state)
        if (m_initialTasksSize == 0)
        {
            AZ_Warning("ApplePicker", false, "ReportProgress reporting 1 since no apples were found in the call");
            return 1.0f;
        }

        return 1.0f - (m_currentAppleTasks.size() / m_initialTasksSize);
    }

    void ApplePickerComponent::Activate()
    {
        if (!m_effectorEntityId.IsValid())
        {
            AZ_Warning("ApplePicker", false, "Effector entity not set, assuming same entity");
            m_effectorEntityId = GetEntityId();
        }
        ApplePickingNotificationBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();

        auto ros2Node = ROS2Interface::Get()->GetNode();
        auto robotNamespace = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity())->GetNamespace();
        auto topic = ROS2Names::GetNamespacedName(robotNamespace, m_triggerServiceTopic);
        m_triggerService = ros2Node->create_service<std_srvs::srv::Trigger>(
            topic.c_str(),
            [this](const TriggerRequest request, TriggerResponse response)
            {
                this->ProcessTriggerServiceCall(request, response);
            });
    }

    void ApplePickerComponent::Deactivate()
    {
        m_triggerService.reset();
        AZ::TickBus::Handler::BusDisconnect();
        ApplePickingNotificationBus::Handler::BusDisconnect();
    }

    void ApplePickerComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ApplePickerComponent, AZ::Component>()
                ->Version(2)
                ->Field("TriggerServiceTopic", &ApplePickerComponent::m_triggerServiceTopic)
                ->Field("EffectorEntity", &ApplePickerComponent::m_effectorEntityId)
                ->Field("FruitStorageEntity", &ApplePickerComponent::m_fruitStorageEntityId);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ApplePickerComponent>("Apple picking component", "A demo component for apple picking")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "AppleKraken")
                    ->DataElement(
                        AZ::Edit::UIHandlers::EntityId,
                        &ApplePickerComponent::m_triggerServiceTopic,
                        "Trigger",
                        "ROS2 service name for gathering trigger")
                    ->DataElement(
                        AZ::Edit::UIHandlers::EntityId,
                        &ApplePickerComponent::m_effectorEntityId,
                        "Effector",
                        "Effector (manipulator) entity")
                    ->DataElement(
                        AZ::Edit::UIHandlers::EntityId,
                        &ApplePickerComponent::m_fruitStorageEntityId,
                        "Fruit Storage",
                        "Fruit storage entity");
            }
        }
    }

    void ApplePickerComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC("ROS2Frame"));
    }

    void ApplePickerComponent::EffectorReadyForPicking()
    {
        PickNextApple();
    }

    void ApplePickerComponent::ApplePicked()
    {
        if (m_currentAppleTasks.empty())
        {
            AZ_Error("ApplePicker", false, "ApplePicked called but no current task");
            return;
        }

        AZ_TracePrintf("ApplePicker", "%s. Picked apple\n", Internal::CurrentTaskString(m_currentAppleTasks).c_str());

        // Disappear the apple
        AZ::Render::MeshComponentRequestBus::Event(
            m_currentAppleTasks.front().m_appleEntityId, &AZ::Render::MeshComponentRequestBus::Events::SetVisibility, false);
    }

    void ApplePickerComponent::AppleRetrieved()
    {
        if (m_currentAppleTasks.empty())
        {
            AZ_Error("ApplePicker", false, "AppleRetrieved called but no current task");
            return;
        }
        AZ_TracePrintf(
            "ApplePicker", "%s. An apple has been retrieved and stored\n", Internal::CurrentTaskString(m_currentAppleTasks).c_str());
        m_currentAppleTasks.pop();

        if (!m_fruitStorageEntityId.IsValid())
        {
            AZ_Warning("ApplePicker", false, "Fruit storage entity not set, assuming same entity");
            m_fruitStorageEntityId = GetEntityId();
        }
        Tags applePickingEventTags = { "automated_picking" };
        FruitStorageRequestsBus::Event(m_fruitStorageEntityId, &FruitStorageRequests::AddApple, applePickingEventTags);
        PickNextApple();
    }

    void ApplePickerComponent::PickingFailed(const AZStd::string& reason)
    { // TODO - refactor common code (debugs, checks)
        if (m_currentAppleTasks.empty())
        {
            AZ_Error("ApplePicker", false, "PickingFailed called but no current task");
            return;
        }
        AZ_TracePrintf(
            "ApplePicker", "%s. Picking failed due to: %s\n", Internal::CurrentTaskString(m_currentAppleTasks).c_str(), reason.c_str());
        m_currentAppleTasks.pop();
        PickNextApple();
    }

    void ApplePickerComponent::PickNextApple()
    {
        AZ_TracePrintf("ApplePicker", "Pick next apple");
        if (!m_currentAppleTasks.empty())
        { // Get another apple!
            ApplePickingRequestBus::Event(m_effectorEntityId, &ApplePickingRequests::PickApple, m_currentAppleTasks.front());
            return;
        }
        AZ_TracePrintf("ApplePicker", "No more apples!");
        ApplePickingRequestBus::Event(m_effectorEntityId, &ApplePickingRequests::FinishPicking);
    }

    void ApplePickerComponent::QueryEnvironmentForAllApplesInBox(const AZ::Obb& globalBox)
    {
        // TODO - query environment

        // Scene query for `apple` entity, we want visible entities with exact 'Apple' name
        auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();

        // clear
        m_currentAppleTasks = AZStd::queue<PickAppleTask>();
        // apples need to be unique
        AZStd::unordered_set<AZ::EntityId> found_apples;

        for (auto& physicScene : physicsSystem->GetAllScenes())
        {
            if (!physicScene)
            {
                continue;
            }

            AzPhysics::OverlapRequest request = AzPhysics::OverlapRequestHelpers::CreateBoxOverlapRequest(
                2.0f * globalBox.GetHalfLengths(),
                AZ::Transform::CreateFromQuaternionAndTranslation(globalBox.GetRotation(), globalBox.GetPosition()));
            // we want maximum overlap buffer set in `physxsystemconfiguration.setreg`
            request.m_maxResults = physicsSystem->GetConfiguration()->m_overlapBufferSize;
            AzPhysics::SceneQueryHits results = physicScene->QueryScene(&request);
            for (const auto& r : results.m_hits)
            {
                AZStd::string entity_name;
                AZ::ComponentApplicationBus::BroadcastResult(entity_name, &AZ::ComponentApplicationRequests::GetEntityName, r.m_entityId);
                if (entity_name != "Apple")
                {
                    continue;
                }
                if (found_apples.contains(r.m_entityId))
                {
                    continue;
                }
                bool is_visible = false;
                AZ::Render::MeshComponentRequestBus::EventResult(
                    is_visible, r.m_entityId, &AZ::Render::MeshComponentRequests::GetVisibility);
                if (!is_visible)
                {
                    continue;
                }
                AZ::Transform targetTM = AZ::Transform::CreateIdentity();
                AZ::TransformBus::EventResult(targetTM, r.m_entityId, &AZ::TransformBus::Events::GetWorldTM);
                PickAppleTask t;
                t.m_appleEntityId = r.m_entityId;
                t.m_appleBoundingBox = r.m_shape->GetAabb(targetTM);
                t.m_middle = targetTM.GetTranslation(); /// TODO consider `r.m_position` here
                m_currentAppleTasks.push(t);
                found_apples.emplace(r.m_entityId);
            }
        }
        m_initialTasksSize = m_currentAppleTasks.size();
        AZ_Printf("ApplePickerComponent", "There are %d apples in reach box \n", m_currentAppleTasks.size());
    }

} // namespace AppleKraken