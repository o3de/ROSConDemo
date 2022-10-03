/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "ApplePickingRequests.h"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/EBus/EBus.h>

namespace AppleKraken
{
    //! Component for apple picking effector (manipulator)
    class KrakenEffectorComponent
        : public AZ::Component
        , public ApplePickingRequestBus::Handler
        , public AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(KrakenEffectorComponent, "{9206FC30-DF56-4246-8247-5D6B31603B53}");
        KrakenEffectorComponent() = default;
        static void Reflect(AZ::ReflectContext* context);

    private:
        void Activate() override;
        void Deactivate() override;

        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        void PrepareForPicking() override;
        void PickApple(const PickAppleTask& appleTask) override;
        void FinishPicking() override;
        PickingState GetEffectorState() override;
        AZ::Obb GetEffectorReachArea() override;

        void BeginTransitionIfAcceptable(EffectorState targetState);
        bool IsTransitionAcceptable(EffectorState targetState) const;
        bool IsTransitionValid(EffectorState targetState) const;

        void OnEffectorReadyForPicking();
        void OnApplePicked();
        void OnAppleRetrieved();

        EffectorState m_effectorState = EffectorState::IDLE;
        EffectorState m_effectorTargetState = EffectorState::IDLE;
        float m_currentStateTransitionTime = 0.0f;
    };
} // namespace AppleKraken
