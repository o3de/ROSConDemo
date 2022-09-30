/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include <AzCore/Serialization/SerializeContext.h>
#include <Atom/RPI.Reflect/Asset/AssetUtils.h>
#include <Atom/RPI.Public/RenderPipeline.h>
#include <Atom/RPI.Public/Pass/PassFilter.h>
#include <Atom/RPI.Public/Pass/PassSystemInterface.h>

#include "SunShaftsFeatureProcessor.h"

namespace ROSConDemo
{
    SunShaftsFeatureProcessor::SunShaftsFeatureProcessor()
    {
    }

    SunShaftsFeatureProcessor::~SunShaftsFeatureProcessor()
    {
    }

    void SunShaftsFeatureProcessor::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext
                ->Class<SunShaftsFeatureProcessor, AZ::RPI::FeatureProcessor>()
                ->Version(1);
        }
    }

    void SunShaftsFeatureProcessor::Activate()
    {
        EnableSceneNotification();
    }

    void SunShaftsFeatureProcessor::Deactivate()
    {
        DisableSceneNotification();
    }

    void AddPassRequestToRenderPipeline(
        AZ::RPI::RenderPipeline* renderPipeline,
        const char* passRequestAssetFilePath,
        const char* referencePass, bool beforeReferencePass)
    {
        auto passRequestAsset = AZ::RPI::AssetUtils::LoadAssetByProductPath<AZ::RPI::AnyAsset>(
            passRequestAssetFilePath, AZ::RPI::AssetUtils::TraceLevel::Warning);
        const AZ::RPI::PassRequest* passRequest = nullptr;
        if (!passRequestAsset)
        {
            AZ_Error("SunShafts", false, "Failed to load PassRequestAsset from %s", passRequestAssetFilePath);
            return;
        }
        if (passRequestAsset->IsReady())
        {
            passRequest = passRequestAsset->GetDataAs<AZ::RPI::PassRequest>();
        }
        if (!passRequest)
        {
            AZ_Error("SunShafts", false, "Can't load PassRequest from %s", passRequestAssetFilePath);
            return;
        }

        // Return if the pass to be created already exists
        AZ::RPI::PassFilter passFilter = AZ::RPI::PassFilter::CreateWithPassName(passRequest->m_passName, renderPipeline);
        AZ::RPI::Pass* existingPass = AZ::RPI::PassSystemInterface::Get()->FindFirstPass(passFilter);
        if (existingPass)
        {
            return;
        }

        // Create the pass
        AZ::RPI::Ptr<AZ::RPI::Pass> newPass = AZ::RPI::PassSystemInterface::Get()->CreatePassFromRequest(passRequest);
        if (!newPass)
        {
            AZ_Error("SunShafts", false, "Failed to create the pass from pass request [%s].", passRequest->m_passName.GetCStr());
            return;
        }

        // Add the pass to render pipeline
        bool success;
        if (beforeReferencePass)
        {
            success = renderPipeline->AddPassBefore(newPass, AZ::Name(referencePass));
        }
        else
        {
            success = renderPipeline->AddPassAfter(newPass, AZ::Name(referencePass));
        }
        // only create pass resources if it was success
        if (!success)
        {
            AZ_Error(
                "SunShafts", false, "Failed to add pass [%s] to render pipeline [%s].", newPass->GetName().GetCStr(),
                renderPipeline->GetId().GetCStr());
        }
    }

    void SunShaftsFeatureProcessor::ApplyRenderPipelineChange(AZ::RPI::RenderPipeline* renderPipeline)
    {
        AddPassRequestToRenderPipeline(renderPipeline, "Passes/SunShaftsFullScreenPassRequest.azasset", "SkyBoxPass", /*before*/ true);
    }

    void SunShaftsFeatureProcessor::Simulate(const FeatureProcessor::SimulatePacket& packet)
    {
        AZ_PROFILE_FUNCTION(AzRender);
        AZ_UNUSED(packet);
    }

    void SunShaftsFeatureProcessor::Render([[maybe_unused]] const FeatureProcessor::RenderPacket& packet)
    {
        AZ_PROFILE_FUNCTION(AzRender);
    }

    void SunShaftsFeatureProcessor::OnRenderPipelineAdded([[maybe_unused]] AZ::RPI::RenderPipelinePtr renderPipeline)
    {
    }

    void SunShaftsFeatureProcessor::OnRenderPipelineRemoved([[maybe_unused]] AZ::RPI::RenderPipeline* renderPipeline)
    {
    }

    void SunShaftsFeatureProcessor::OnRenderPipelinePassesChanged([[maybe_unused]] AZ::RPI::RenderPipeline* renderPipeline)
    {
    }

} // namespace ROSConDemo

//#pragma optimize("", on)
