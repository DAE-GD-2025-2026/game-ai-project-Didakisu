#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

//SEEK
//*******
// TODO: Do the Week01 assignment :^)

SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    SteeringOutput Steering{};

    Steering.LinearVelocity = Target.Position - Agent.GetPosition();

    return Steering;
}

SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    SteeringOutput Steering{};

    Steering.LinearVelocity = Agent.GetPosition() - Target.Position;

    return Steering;
}

SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    SteeringOutput Steering{};

    if (m_OriginalMaxSpeed < 0.f)
    {
        m_OriginalMaxSpeed = Agent.GetMaxLinearSpeed();
    }

    FVector2D toTarget = Target.Position - Agent.GetPosition();
    float distance = toTarget.Length();

    if (distance <= m_TargetRadius)
    {
        Agent.SetMaxLinearSpeed(0.f);
    }
    else if (distance >= m_SlowRadius)
    {
        Agent.SetMaxLinearSpeed(m_OriginalMaxSpeed);
    }
    else
    {
        float ratio = (distance - m_TargetRadius) / (m_SlowRadius - m_TargetRadius);
        Agent.SetMaxLinearSpeed(m_OriginalMaxSpeed * ratio);
    }

    Steering.LinearVelocity = toTarget;

    return Steering;
}