
#pragma once

#include <Atom/RPI.Public/FeatureProcessor.h>

namespace ROSConDemo
{
    class SunShaftsFeatureProcessor final
        : public AZ::RPI::FeatureProcessor
    {
    public:
        AZ_RTTI(ROSConDemo::SunShaftsFeatureProcessor, "{C5775AB6-D38D-44C4-A526-A4F9E0BC7B8B}", AZ::RPI::FeatureProcessor);

        static void Reflect(AZ::ReflectContext* context);

        SunShaftsFeatureProcessor();
        virtual ~SunShaftsFeatureProcessor();

        //! RPI::FeatureProcessor
        void Activate() override;
        void Deactivate() override;
        void ApplyRenderPipelineChange(AZ::RPI::RenderPipeline* renderPipeline) override;
        void Simulate(const FeatureProcessor::SimulatePacket& packet) override;
        void Render(const FeatureProcessor::RenderPacket& packet) override;

        //! RPI::SceneNotificationBus
        void OnRenderPipelineAdded(AZ::RPI::RenderPipelinePtr renderPipeline) override;
        void OnRenderPipelineRemoved(AZ::RPI::RenderPipeline* renderPipeline) override;
        void OnRenderPipelinePassesChanged(AZ::RPI::RenderPipeline* renderPipeline) override;

    private:
        AZ_DISABLE_COPY_MOVE(SunShaftsFeatureProcessor);
    };
}
