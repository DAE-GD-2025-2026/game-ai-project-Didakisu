#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"
#include "Engine/World.h"
#include "DrawDebugHelpers.h"

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
    const UWorld* InWorld = Agent.GetWorld();

    if (m_OriginalMaxSpeed < 0.f)
    {
        m_OriginalMaxSpeed = Agent.GetMaxLinearSpeed();
    }

    FVector yAxis = FVector(1, 0, 0);
    FVector zAxis = FVector(0, 1, 0);
    FVector targetPoint = FVector(Target.Position.X , Target.Position.Y , 0);

    FVector2D toTarget = Target.Position - Agent.GetPosition();
    float distance = toTarget.Length();
    FVector center = { Agent.GetPosition().X , Agent.GetPosition().Y , 0};

    DrawDebugCircle(InWorld, center, m_SlowRadius, 32, FColor::Blue, false, -1.0f, 0, 0.f, yAxis , zAxis);
    DrawDebugCircle(InWorld, center, m_TargetRadius, 32, FColor::Red, false, -1.0f, 0, 0.f , yAxis , zAxis);
    DrawDebugPoint(InWorld, targetPoint , 10, FColor::Magenta);

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
    const UWorld* InWorld = Agent.GetWorld();

    FVector2D toTarget = Target.Position - Agent.GetPosition();
    float distance = toTarget.Length();

    auto t = distance / Agent.GetMaxLinearSpeed(); //time to reach the target based on agent's(pursuer) speed

    auto predictedPos = Target.Position + Target.LinearVelocity * t;

    auto direction = predictedPos - Agent.GetPosition();
    Steering.LinearVelocity = direction;

    FVector lineStart = { Agent.GetPosition().X , Agent.GetPosition().Y , 0 };
    FVector lineEnd = { predictedPos.X , predictedPos.Y , 0 };

    DrawDebugLine(InWorld , lineStart , lineEnd , FColor::Cyan);

    return Steering;
}

SteeringOutput Evade::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    SteeringOutput Steering{};

    FVector2D toTarget = Target.Position - Agent.GetPosition();
    float distance = toTarget.Length();

    auto t = distance / Agent.GetMaxLinearSpeed(); //time to reach the target based on agent's(pursuer) speed

    auto predictedPos = Target.Position + Target.LinearVelocity * t;

    auto direction = Agent.GetPosition() - predictedPos;
    Steering.LinearVelocity = direction;

    return Steering;
}

SteeringOutput Wander::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    SteeringOutput Steering{};

    const UWorld* InWorld = Agent.GetWorld();
    
    float radius = 100.0f;
    const FColor color = FColor::Red;

    FVector yAxis = FVector(1,0,0);
    FVector zAxis = FVector(0,1,0);

    FVector forwardDirection = Agent.GetVelocity().GetSafeNormal(); //get the direction of the agent
    FVector2D realForwardDirection = { forwardDirection.X , forwardDirection.Y};

    FVector2D point = Agent.GetPosition() + (realForwardDirection * m_ForwardDistance);
    FVector threeDPoint = { point.X , point.Y , 50.f };

    DrawDebugCircle(InWorld, threeDPoint, radius, 32, color, false, -1.0f, 0, 0.f, yAxis, zAxis);

    double theta = ((double)rand() / RAND_MAX) * 2.0 * PI; //get a random num betwen 0 and 2pi

    FVector2D offset;//creating a point that lies on the circle
    offset.X = radius * cos(theta);
    offset.Y = radius * sin(theta);

    auto wanderTarget = offset + point;
    auto direction = wanderTarget - Agent.GetPosition();

    FVector lineStart = { Agent.GetPosition().X , Agent.GetPosition().Y , 0 };
    FVector lineEnd = { wanderTarget.X , wanderTarget.Y , 0 };

    DrawDebugLine(InWorld, lineStart, lineEnd, color, false, -1.0f, 0, 0.0f );//the vector pointing from the agent to the wander traget
    DrawDebugPoint(InWorld, lineEnd, 10, FColor::Green);

    Steering.LinearVelocity = direction;

    return Steering;
}
