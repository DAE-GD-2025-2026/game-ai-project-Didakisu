// Fill out your copyright notice in the Description page of Project Settings.


#include "Level_GraphTheory.h"

#include "Algorithms/EulerianPath.h"
#include "Shared/GameAISpectator.h"

using namespace GameAI;

// Sets default values
ALevel_GraphTheory::ALevel_GraphTheory()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ALevel_GraphTheory::BeginPlay()
{
	Super::BeginPlay();
	Renderer = GraphRenderer(GetWorld());
	
	// Add the graph editor to our player
	if (PlayerController = Cast<APlayerController>(GetWorld()->GetFirstLocalPlayerFromController()->PlayerController); 
		GraphEditorClass && PlayerController)
	{
		PlayerGraphEditor = NewObject<UGraphEditorComponent>(PlayerController->GetPawn(), GraphEditorClass);
		PlayerGraphEditor->RegisterComponent();
		PlayerGraphEditor->SetEditedGraph(&Graph);
		PlayerGraphEditor->SetNodeFactory(&NodeFactory);
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("Unable to get PlayerController from LocalPlayer or GraphEditorClass is null"))
		return;
	}
	
	// Make the view orthogonal for less perspective issues
	if (AGameAISpectator* Player = Cast<AGameAISpectator>(PlayerController->GetPawnOrSpectator()); Player)
	{
		Player->SetCameraProjection(ECameraProjectionMode::Orthographic);
	}
		
	auto indx1 = Graph.AddNode(NodeFactory.CreateNode(FVector2D(-200.f, -400.f)));
	auto indx2 = Graph.AddNode(NodeFactory.CreateNode(FVector2D(-200.f, 100.f)));
	auto indx3 = Graph.AddNode(NodeFactory.CreateNode(FVector2D(300.f, 100.f)));
	auto indx4 = Graph.AddNode(NodeFactory.CreateNode(FVector2D(300.f, -400.f)));
	auto indx5 = Graph.AddNode(NodeFactory.CreateNode(FVector2D(580.f, -155.f)));

	Graph.AddConnection(std::make_unique<GameAI::Connection>(indx1, indx2));
	Graph.AddConnection(std::make_unique<GameAI::Connection>(indx2, indx3));
	Graph.AddConnection(std::make_unique<GameAI::Connection>(indx3, indx4));
	Graph.AddConnection(std::make_unique<GameAI::Connection>(indx4, indx5));
	Graph.AddConnection(std::make_unique<GameAI::Connection>(indx1, indx4));


	Graph.AddConnection(std::make_unique<GameAI::Connection>(indx1, indx3));
	Graph.AddConnection(std::make_unique<GameAI::Connection>(indx3, indx5));
	Graph.AddConnection(std::make_unique<GameAI::Connection>(indx2, indx4));

	// Spawn the Agent
	Agent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass,
		FVector{ -200, -400 ,90 }, FRotator::ZeroRotator);

	Agent->SetSteeringBehavior(&PathFollow);

	/*EulerianPath eulerianPath(&Graph);
	Eulerianity eulerianity{};
	auto path = eulerianPath.FindPath(eulerianity);

	if (!path.empty())
	{
		UpdateAgentPath(path);
	}*/
}

void ALevel_GraphTheory::BeginDestroy()
{
	Super::BeginDestroy();
}

void ALevel_GraphTheory::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
#pragma region UI
	{
		//Setup
		bool windowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Gameplay Programming", &windowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
		ImGui::SetWindowFocus();
		ImGui::PushItemWidth(70);
		//Elements
		ImGui::Text("CONTROLS");
		ImGui::Indent();
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		ImGui::Spacing();

		ImGui::Text("STATS");
		ImGui::Indent();
		ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
		ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		ImGui::Spacing();

		ImGui::Text("Graph Theory");
		ImGui::Spacing();
		ImGui::Spacing();

		//End
		ImGui::End();
	}
#pragma endregion UI
	
	Renderer.RenderGraph(Graph);
	
	if (PlayerGraphEditor->HasGraphUpdated())
	{
		for (auto& node : Graph.GetNodes())
		{
			if (node->GetId() != GameAI::Graphs::InvalidNodeId)
			{
				UE_LOG(LogTemp, Warning, TEXT("Node %d: X=%f Y=%f"),
					node->GetId(),
					node->GetPosition().X,
					node->GetPosition().Y);
			}
		}

		EulerianPath eulerianPath(&Graph);
		Eulerianity eulerianity{};
		auto path = eulerianPath.FindPath(eulerianity);

		if (!path.empty())
		{
			UpdateAgentPath(path);
		}
	}
}

void ALevel_GraphTheory::UpdateAgentPath(std::vector<Node*> const& Trail)
{
	std::vector<FVector2D> path{};
	
	for (int i = 0; i < Trail.size(); i++)
	{
		path.push_back(Trail[i]->GetPosition());
	}

	PathFollow.SetPath(path);
	if (path.size() > 0)
	{
		Agent->SetPosition(path[0]);
	}
}