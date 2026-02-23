#include "FlockingSteeringBehaviors.h"
#include "Flock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"


//*******************
//COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
    SteeringOutput Steering{};

    auto cohesionVector = pFlock->GetAverageNeighborPos() - pAgent.GetPosition();
    Steering.LinearVelocity = cohesionVector;

    return Steering;
}

//*********************
//SEPARATION (FLOCKING)
SteeringOutput Separation::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
    SteeringOutput Steering{};

    //get the vector from any neighbors to the agent
    auto neighbors = pFlock->GetNeighbors();
    FVector2D totalPushes = FVector2D::ZeroVector;

    for (int i = 0; i < neighbors.Num(); i++)
    {
        if (!neighbors[i]) continue;
        FVector2D awayVec = pAgent.GetPosition() - neighbors[i]->GetPosition();//points from the neighbor to the agent 
        //the closer the neighbor, the stronger the push away
        //close neighbor == big push
        //far neighbor == small push
        
        //and when the distance is big (for example 10) => 1/10 = 0.1, which is small push
        float distance = FMath::Max(awayVec.Length(), 0.1f);//avoiding division by zero
        totalPushes += awayVec.GetSafeNormal() / distance;
       
        //the agent must feel the pushes from each neighbor together, so we add all puehs
    }
    //and then scale that vector inversely to how close it is
    Steering.LinearVelocity = totalPushes.GetSafeNormal() * pAgent.GetMaxLinearSpeed();

    return Steering;
}

//*************************
//VELOCITY MATCH (FLOCKING)

SteeringOutput VelocityMatch::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
    SteeringOutput Steering{};

    auto avgVelocity = pFlock->GetAverageNeighborVelocity();
    FVector2D agentVelocity = { pAgent.GetVelocity().X , pAgent.GetVelocity().Y };
    Steering.LinearVelocity = avgVelocity - agentVelocity ;

    return Steering;
}