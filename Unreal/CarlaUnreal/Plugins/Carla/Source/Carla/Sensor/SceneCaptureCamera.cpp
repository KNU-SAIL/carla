// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Sensor/SceneCaptureCamera.h"
#include "Carla.h"
#include "Carla/Game/CarlaEngine.h"
#include <chrono>

#include <util/ue-header-guard-begin.h>
#include "Actor/ActorBlueprintFunctionLibrary.h"
#include "RenderingThread.h"
#include <util/ue-header-guard-end.h>

FActorDefinition ASceneCaptureCamera::GetSensorDefinition()
{
    constexpr bool bEnableModifyingPostProcessEffects = true;
    return UActorBlueprintFunctionLibrary::MakeCameraDefinition(
        TEXT("rgb"),
        bEnableModifyingPostProcessEffects);
}

ASceneCaptureCamera::ASceneCaptureCamera(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
    AddPostProcessingMaterial(
        TEXT("Material'/Carla/PostProcessingMaterials/PhysicLensDistortion.PhysicLensDistortion'"));
}

void ASceneCaptureCamera::BeginPlay()
{
  Super::BeginPlay();
}

void ASceneCaptureCamera::OnFirstClientConnected()
{
}

void ASceneCaptureCamera::OnLastClientDisconnected()
{
}

void ASceneCaptureCamera::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
  Super::EndPlay(EndPlayReason);
}

void ASceneCaptureCamera::PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaSeconds)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ASceneCaptureCamera::PostPhysTick);
  Super::PostPhysTick(World, TickType, DeltaSeconds);
  
  ENQUEUE_RENDER_COMMAND(MeasureTime)
  (
    [](auto &InRHICmdList)
    {
      std::chrono::time_point<std::chrono::high_resolution_clock> Time = 
          std::chrono::high_resolution_clock::now();
      auto Duration = std::chrono::duration_cast< std::chrono::milliseconds >(Time.time_since_epoch());
      uint64_t Milliseconds = Duration.count();
      FString ProfilerText = FString("(Render)Frame: ") + FString::FromInt(FCarlaEngine::GetFrameCounter()) + 
          FString(" Time: ") + FString::FromInt(Milliseconds);
      TRACE_CPUPROFILER_EVENT_SCOPE_TEXT(*ProfilerText);
    }
  );

  if (!AreClientsListening())
      return;

  auto FrameIndex = FCarlaEngine::GetFrameCounter();
  const bool bMono8 = (GetEncoding() == EEncoding::MONO8);
  ImageUtil::ReadSensorImageDataAsyncFColor(*this, [this, FrameIndex, bMono8](
    TArrayView<const FColor> Pixels,
    FIntPoint Size) -> bool
  {
    if (bMono8)
    {
      // BGRA -> Grayscale FColor (ITU-R BT.601): R=G=B=0.2989*R+0.587*G+0.114*B
      // Always send FColor so native clients (PythonAPI) can deserialize as Array<Color>.
      // ROS2 publisher extracts the mono channel before publishing.
      TArray<FColor> GrayPixels;
      GrayPixels.SetNumUninitialized(Pixels.Num());
      for (int32 i = 0; i < Pixels.Num(); ++i)
      {
        const FColor& C = Pixels[i];
        uint8 Gray = static_cast<uint8>(0.2989f * C.R + 0.587f * C.G + 0.114f * C.B);
        GrayPixels[i] = FColor(Gray, Gray, Gray, 255);
      }
      SendDataToClient(*this, TArrayView<const FColor>(GrayPixels.GetData(), GrayPixels.Num()), FrameIndex);
    }
    else
    {
      SendDataToClient(*this, Pixels, FrameIndex);
    }
    return true;
  });
}

#ifdef CARLA_HAS_GBUFFER_API
void ASceneCaptureCamera::SendGBufferTextures(FGBufferRequest& GBuffer)
{
    SendGBufferTexturesInternal(*this, GBuffer);
}
#endif
