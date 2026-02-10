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

SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    SteeringOutput Steering{};

    auto direction = Target.Position - Agent.GetPosition();//get direction
    auto desiredAngle = atan2(direction.Y , direction.X);//rad
    auto currentOrientation = FMath::DegreesToRadians(Agent.GetActorRotation().Yaw);//yaw gives degrees
    auto angleDifference = desiredAngle - currentOrientation;

    Steering.AngularVelocity = FMath::RadiansToDegrees(angleDifference);

    auto threshold = FMath::DegreesToRadians(1.f);

    if (FMath::Abs(angleDifference) > threshold)
    {
        //apply linear velocity
        direction.Normalize();//if we don't normalize it will move faster than 0,01
        Steering.LinearVelocity = direction * 0.01f;
    }
    else //if it is already facing the target stop linear velocity
    {
        Steering.LinearVelocity = FVector2D::ZeroVector;
    }

    return Steering;
}

SteeringOutput Pursuit::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    //agent is pursuer
    SteeringOutput Steering{};

    FVector2D toTarget = Target.Position - Agent.GetPosition();
    float distance = toTarget.Length();

    auto t = distance / Agent.GetMaxLinearSpeed(); //time to reach the target based on agent's(pursuer) speed

    auto predictedPos = Target.Position + Target.LinearVelocity * t;

    auto direction = predictedPos - Agent.GetPosition();
    Steering.LinearVelocity = direction;

    return Steering;
}
