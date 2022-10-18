/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GatheringRowComponent.h"
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>

namespace AppleKraken
{
    void GatheringRowComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<GatheringRowComponent, AZ::Component>()
                ->Version(4)
                ->Field("PoseOffset", &GatheringRowComponent::m_poseOffset);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<GatheringRowComponent>("Gathering Row Component", "Poses (points with orientation) suitable for apple gathering."
                                                                            "Component will return as navigation plan all its children "
                                                                            "with name containing \'GatherPoint\'. "
                                                                            "Points are sorted with entity name.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "AppleKraken")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GatheringRowComponent::m_poseOffset,
                        "Offset",
                        "Pose offset for each point (depends on robot)")
                    ->Attribute(AZ::Edit::Attributes::Min, 1);
            }
        }
    }

    void GatheringRowComponent::Activate()
    {
        GatheringRowRequestBus::Handler::BusConnect();
    }

    void GatheringRowComponent::Deactivate()
    {
        GatheringRowRequestBus::Handler::BusDisconnect();
        m_gatheringPoses.clear();
    }

    void GatheringRowComponent::ComputeGatheringPoses()
    {
        // find all children
        AZStd::vector<AZ::EntityId> descendants;
        AZ::TransformBus::EventResult(descendants, GetEntityId(), &AZ::TransformBus::Events::GetAllDescendants);
        if (descendants.empty())
        {
            AZ_Error("GatheringRowComponent", false, "ComputeGatheringPoses: unable to proceed with empty set");
            return;
        }
        // Simplification - we assume same orientations along the way
        AZStd::map<AZStd::string, AZ::EntityId> sorted_names;
        for (const auto& entity_id:descendants)
        {
            AZStd::string entity_name;
            AZ::ComponentApplicationBus::BroadcastResult(entity_name, &AZ::ComponentApplicationRequests::GetEntityName, entity_id);
            if (entity_id.IsValid()) {
                if (entity_name.contains("GatherPoint")) {
                    sorted_names[entity_name] = entity_id;
                }
            }
        }
        for (const auto &[_, entity_id]:sorted_names)
        {
            AZ::Transform pose;
            AZ::TransformBus::EventResult(pose, entity_id, &AZ::TransformBus::Events::GetWorldTM);
            pose = pose * AZ::Transform::CreateFromQuaternionAndTranslation(AZ::Quaternion::CreateIdentity(),
                                                                            m_poseOffset);
            m_gatheringPoses.emplace_back(pose);
        }
    }
} // namespace AppleKraken
