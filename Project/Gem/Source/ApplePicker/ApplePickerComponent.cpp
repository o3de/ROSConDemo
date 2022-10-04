/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ApplePickerComponent.h"
#include "ApplePickingRequests.h"
#include <AzCore/EBus/Event.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

#include <AtomLyIntegration/CommonFeatures/Mesh/MeshComponentBus.h>
#include <AtomLyIntegration/CommonFeatures/Mesh/MeshComponentConstants.h>
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/Shape.h>

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

    void ApplePickerComponent::StartAutomatedOperation()
    {
        if (!m_currentAppleTasks.empty())
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

        // TODO - a debug loop
        if (m_currentAppleTasks.empty())
        {
            StartAutomatedOperation();
        }
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
        m_effectorEntityId = GetEntityId(); // TODO - remove this once we expose this field
        ApplePickingNotificationBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();
    }

    void ApplePickerComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        ApplePickingNotificationBus::Handler::BusDisconnect();
    }

    void ApplePickerComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ApplePickerComponent, AZ::Component>()->Version(1);
            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ApplePickerComponent>("Apple picking component", "A demo component for apple picking")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "AppleKraken");
            }
        }
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