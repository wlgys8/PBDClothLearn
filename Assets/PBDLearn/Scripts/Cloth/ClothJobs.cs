using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Collections;

namespace PBDLearn
{
    public struct DistanceConstraintInfo{
        public float restLength;
        public int vIndex0;
        public int vIndex1;
    }

    public struct BendConstraintInfo{
        public int vIndex0;
        public int vIndex1;
        public int vIndex2;
        public int vIndex3;
        public float rest; // angle
    }

    public struct PinConstraintInfo{
        public float3 position;
    }

    /// <summary>
    /// 碰撞约束
    /// </summary>
    public struct CollisionConstraintInfo{
        public float3 concatPosition;
        public float3 normal;
        public float3 velocity;
    }

    [Unity.Burst.BurstCompile]
    struct PositionEstimateJob : IJobParallelFor
    {
        [ReadOnly] 
        public NativeArray<float3> positions;
        [ReadOnly]
        public NativeArray<float3> normals;

        [ReadOnly] 
        public NativeArray<float3> velocities;
        [ReadOnly]
        public NativeArray<float> masses;
        [WriteOnly]
        public NativeArray<float3> predictPositions;
        public float3 fieldForce;
        public float damper;
        public float dt;
        public void Execute(int index)
        {
            var p = positions[index];
            var v = velocities[index];
            var m = masses[index];
            if(m > 0){
                var normal = normals[index];
                var fieldForceAtNormal = math.dot(fieldForce,normal) * normal;
                var v1 = v + ClothSimulator.G * dt + fieldForceAtNormal * dt / m;
                v1 *= math.max(0,(1 - damper * dt / m)); //阻尼
                var p1 = p + v1 * dt;
                predictPositions[index] = p1;
            }else{
                predictPositions[index] = p;
            }
            
        }
    }

    [Unity.Burst.BurstCompile]
    public struct CollisionJob : IJobParallelFor
    {
        [ReadOnly]
        public float dt;
        [ReadOnly]
        public NativeArray<float3> positions;
        [ReadOnly]
        public NativeArray<float3> predictPositions;

        [ReadOnly]
        public NativeArray<float> masses;

        [ReadOnly]
        public CollidersGroup collidersGroup;

        [WriteOnly]
        public NativeArray<CollisionConstraintInfo> collisionConstraints;
        public NativeArray<ConstraintType> constraintTypes;
        public NativeList<RigidBodyForceApply>.ParallelWriter rigidBodyForceApplies;


        private void EnableCollisionConstraint(int index,ref ConcatInfo concatInfo,ref RigidbodyDesc rigidbody,int entityId){
            constraintTypes[index] |= ConstraintType.Collision;

            var collisionConstraintInfo = new CollisionConstraintInfo(){
                concatPosition = concatInfo.position + concatInfo.normal * 0.05f,
                normal = concatInfo.normal
            };
            var m1 = rigidbody.mass;
            if(m1 > 0){
                var bounciness = rigidbody.bounciness;
                var m0 = masses[index];
                var v0 = (predictPositions[index] - positions[index])/dt;
                var v0Normal = math.dot(v0,concatInfo.normal) * concatInfo.normal;
                var v1Normal = math.dot(rigidbody.velocity,concatInfo.normal) * concatInfo.normal;
                
                var v0NormalNew = (bounciness + 1) * m1 * v1Normal + v0Normal * (m0 - bounciness * m1);
                var v1NormalNew = (bounciness + 1) * m0 * v0Normal + v1Normal * (m1 - bounciness * m0);

                v0NormalNew /= (m0 + m1);
                v1NormalNew /= (m0 + m1);

                rigidBodyForceApplies.AddNoResize(new RigidBodyForceApply(){
                    entityId =  entityId,
                    velocity = v1NormalNew - v1Normal
                });
                collisionConstraintInfo.velocity = (v0 - v0Normal + v0NormalNew);
            }

            collisionConstraints[index] = collisionConstraintInfo;
        }

        private void DisableCollisionConstraint(int index){
            constraintTypes[index] &= (~ConstraintType.Collision);
        }


        public void Execute(int index)
        {
            var position = this.positions[index];
            var predictPosition = this.positions[index];
            var spheres = collidersGroup.spheres;
            var boxes = collidersGroup.boxes;
            var capsules = collidersGroup.capsules;

            for(var i = 0; i < spheres.Length; i ++){
                var s = spheres[i];
                ConcatInfo concatInfo;
                if(IntersectUtil.GetClosestSurfacePoint(position,s.collider,out concatInfo)){
                    EnableCollisionConstraint(index,ref concatInfo,ref s.rigidbody,s.entityId);
                    return;
                }
            }

            for(var i = 0; i < boxes.Length; i ++){
                var s = boxes[i];
                ConcatInfo concatInfo;
                if(IntersectUtil.GetClosestSurfacePoint(position,s.collider,out concatInfo)){
                    EnableCollisionConstraint(index,ref concatInfo,ref s.rigidbody,s.entityId);
                   
                    return;
                }
            }

            for(var i = 0; i < capsules.Length; i ++){
                var s = capsules[i];
                ConcatInfo concatInfo;
                if(IntersectUtil.GetClosestSurfacePoint(position,s.collider,out concatInfo)){
                    EnableCollisionConstraint(index,ref concatInfo,ref s.rigidbody,s.entityId);
                    return;
                }
            }
            DisableCollisionConstraint(index);
        }
    }

    /// <summary>
    /// 距离约束任务
    /// </summary>
    [Unity.Burst.BurstCompile]
    public struct DistanceConstraintJob : IJobFor
    {
        [ReadOnly]
        public float compressStiffness;

        [ReadOnly]
        public float stretchStiffness;

        [ReadOnly]
        public NativeArray<float3> predictPositions;

        [ReadOnly]
        public NativeArray<float> masses;
        
        [ReadOnly]
        public NativeArray<DistanceConstraintInfo> distanceConstriants;

        public NativeArray<float3> positionCorrects;

        public float di;

        public void Execute(int index)
        {
            var constraint = distanceConstriants[index];

            var p0 = predictPositions[constraint.vIndex0];
            var p1 = predictPositions[constraint.vIndex1];
            var m0 = masses[constraint.vIndex0];
            var m1 = masses[constraint.vIndex1];
            var distV = p1 - p0;
            var normal = math.normalize(distV);
            var length = math.length(distV);
            var err = length - constraint.restLength;
            float3 correct;
            if(err < 0){
                correct = compressStiffness * normal * err;
            }else{
                correct = stretchStiffness * normal * err;
            }
            var totalM = m0 + m1;
            positionCorrects[constraint.vIndex0] += correct * di * m1 / totalM;
            positionCorrects[constraint.vIndex1] -= correct * di * m0 / totalM;
        }
    }

    [Unity.Burst.BurstCompile]
    /// <summary>
    /// 为每一条边(即一对三角面片)生成关于4顶点的约束信息
    /// </summary>
    public struct BendConstaintsGenerateJob : IJobFor
    {

        [ReadOnly]
        public NativeArray<BendConstraintInfo> bendConstarints;

        [ReadOnly]
        public NativeArray<float3> predictPositions;

        [ReadOnly]
        public NativeArray<float> masses;

        public NativeArray<float3> verticesCorrectResult;

        [ReadOnly]
        public float di;

        [ReadOnly]
        public float bendStiffness;

        public void Execute(int index)
        {
            var cons = bendConstarints[index];
            
            var p1 = predictPositions[cons.vIndex0];
            var p2 = predictPositions[cons.vIndex1] - p1;
            var p3 = predictPositions[cons.vIndex2] - p1;
            var p4 = predictPositions[cons.vIndex3] - p1;
            p1 = 0;
            var n1 = math.normalize(math.cross(p2,p3));
            var n2 = math.normalize(math.cross(p2,p4));

            var d = math.dot(n1,n2);

            var p23Len = math.length(math.cross(p2,p3));
            var p24Len = math.length(math.cross(p2,p4));

            var q3 = (math.cross(p2,n2) + math.cross(n1,p2) * d) / p23Len;
            var q4 = (math.cross(p2,n1) + math.cross(n2,p2) * d) / p24Len;
            var q2 = - (math.cross(p3,n2) + math.cross(n1,p3) * d) / p23Len 
            - (math.cross(p4,n1) + math.cross(n2,p4) * d) / p24Len;
            var q1 = - q2 - q3 - q4;

            var w1 = 1 / masses[cons.vIndex0];
            var w2 = 1 / masses[cons.vIndex1];
            var w3 = 1 / masses[cons.vIndex2];
            var w4 = 1 / masses[cons.vIndex3];

            var sum = w1 * math.lengthsq(q1) 
            + w2 * math.lengthsq(q2) 
            + w3 * math.lengthsq(q3) 
            + w4 * math.lengthsq(q4);

            sum = math.max(0.01f,sum);

            var s = - (math.acos(d) - cons.rest) * math.sqrt(1 - d * d) / sum;

            if(math.isfinite(s)){
                var dp1 = s * w1 * q1 * di * bendStiffness;
                var dp2 = s * w2 * q2 * di * bendStiffness;
                var dp3 = s * w3 * q3 * di * bendStiffness;
                var dp4 = s * w4 * q4 * di * bendStiffness;
                verticesCorrectResult[cons.vIndex0] += dp1;
                verticesCorrectResult[cons.vIndex1] += dp2;
                verticesCorrectResult[cons.vIndex2] += dp3;
                verticesCorrectResult[cons.vIndex3] += dp4;
            }

        }
    }




    /// <summary>
    /// 约束合计
    /// </summary>
    [Unity.Burst.BurstCompile]
    public struct ConstraintsJob:IJobParallelFor{

        /// <summary>
        /// 总共迭代次数
        /// </summary>
        public int iteratorCount;

        /// <summary>
        /// 当前第几次迭代
        /// </summary>
        public int iterateIndex;


        [ReadOnly]
        public NativeArray<float3> positions;

        public NativeArray<float3> predictPositions;


        [ReadOnly]
        public NativeArray<float> masses;

        [ReadOnly]
        public NativeArray<PinConstraintInfo> pinConstriants;


        [ReadOnly]
        public NativeArray<CollisionConstraintInfo> collisionConstraints;

        [ReadOnly]
        public NativeArray<ConstraintType> activeConstraintTypes;

        public NativeArray<float3> positionCorrect;

        public float di;

        public ConstraintsJob(NativeArray<float3> positions, 
        NativeArray<float3> predictPositions,float di){
            this.positions = positions;
            this.predictPositions = predictPositions;
            this.di = di;
            this.pinConstriants = default;
            this.collisionConstraints = default;
            this.activeConstraintTypes = default;
            this.iteratorCount = 1;
            this.iterateIndex = 0;
            this.masses = default;
            this.positionCorrect = default;
        }

        public void Execute(int index)
        {

            var constraintType = activeConstraintTypes[index];
            var positionCorrect = this.positionCorrect[index];
            this.positionCorrect[index] = 0;
            if((constraintType & ConstraintType.Pin) == ConstraintType.Pin){
                //固定约束
                this.predictPositions[index] = this.pinConstriants[index].position;
                return;
            }
            if((constraintType & ConstraintType.Collision) == ConstraintType.Collision){
                //碰撞约束
                var position = this.positions[index];
                var collisionInfo = collisionConstraints[index];
                var deltaP = collisionInfo.concatPosition - position;
                position += deltaP * (iterateIndex * 1f / iteratorCount);
                predictPositions[index] = position;
                return;
            }
            //其他之前计算好的约束修正
            this.predictPositions[index] += positionCorrect;
        }

    }

    [Unity.Burst.BurstCompile]
    public struct UpdateVelocitiesAndPositionsJob : IJobParallelFor
    {
        [ReadOnly]
        public NativeArray<ConstraintType> constraintTypes;

        [ReadOnly]
        public NativeArray<float3> predictPositions;

        [ReadOnly]
        public NativeArray<CollisionConstraintInfo> collisionConstraintInfos;

        public NativeArray<float3> positions;

        [WriteOnly]
        public NativeArray<float3> velocities;

        public float dt;

        public void Execute(int index)
        {
            if( (constraintTypes[index] & ConstraintType.Collision) == ConstraintType.Collision){
                velocities[index] = this.collisionConstraintInfos[index].velocity;
            }else{
                velocities[index] = (predictPositions[index] - positions[index])/dt;
            }
            positions[index] = predictPositions[index];
        }
    }


    [Unity.Burst.BurstCompile]
    public struct NormalCalulateJob : IJob
    {
        [ReadOnly]
        public NativeArray<int> indices;
        [ReadOnly]
        public NativeArray<float3> positions;

        public NativeArray<float3> normals;

        public void Execute()
        {
            for(var i = 0; i < normals.Length; i ++){
                normals[i] = 0;
            }

            for(var i = 0; i < indices.Length / 3; i ++){
                var offset = i * 3;
                var vIndex0 = indices[offset];
                var vIndex1 = indices[offset + 1];
                var vIndex2 = indices[offset + 2];
                var p0 = positions[vIndex0];
                var p1 = positions[vIndex1];
                var p2 = positions[vIndex2];
                var n = math.normalize(math.cross(p1 - p0,p2 - p0));
                normals[vIndex0] += n;
                normals[vIndex1] += n;
                normals[vIndex2] += n;
            }

            for(var i = 0; i < normals.Length; i ++){
                normals[i] = math.normalizesafe(normals[i]);
            }
        }
    }
}
