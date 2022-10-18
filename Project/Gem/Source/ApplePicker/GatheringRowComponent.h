/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "GatheringRowRequests.h"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityId.h>

namespace AppleKraken
{
    //! Demo component wrapping information about gathering positions (single row of apple trees)
    class GatheringRowComponent
        : public AZ::Component
        , public GatheringRowRequestBus::Handler
    {
    public:
        AZ_COMPONENT(GatheringRowComponent, "{32987AAB-D275-40E0-B9BC-A8108522C055}", AZ::Component);
        GatheringRowComponent() = default;
        static void Reflect(AZ::ReflectContext* context);

        //! First on the list is always the start pose, the last is the end pose
        GatheringPoses GetGatheringPoses() override
        {
            if (m_gatheringPoses.empty())
            {
                ComputeGatheringPoses();
            }
            return m_gatheringPoses;
        }

    private:
        void Activate() override;
        void Deactivate() override;

        void ComputeGatheringPoses();

        AZ::Vector3 m_poseOffset = AZ::Vector3::CreateZero(); //!< Depends on robot / effector setting, exposed for experimenting
        GatheringPoses m_gatheringPoses;
    };
} // namespace AppleKraken
