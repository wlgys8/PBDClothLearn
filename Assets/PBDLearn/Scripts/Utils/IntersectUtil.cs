using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

namespace PBDLearn
{
    public class IntersectUtil
    {

        /// <summary>
        /// 计算三角形面积
        /// </summary>
        public static float GetArea(float3 p0,float3 p1,float3 p2){
            var v0 = p1 - p0;
            var v1 = p2 - p0;
            return math.length(math.cross(v0,v1)) * 0.5f;
        }

        /// <summary>
        /// 检查点是否在球内
        /// </summary>
        public static bool PointInside(float3 p, SphereDesc sphere){
            float3 d = sphere.center - p;
            return math.dot(d,d) < sphere.radius * sphere.radius;
        }

        public static bool GetClosestSurfacePoint(float3 p, SphereDesc sphere, out ConcatInfo concatInfo){
            concatInfo = default(ConcatInfo);
            float3 c2p = p - sphere.center;
            float d2 = math.dot(c2p,c2p);
            float r2 = sphere.radius * sphere.radius;
            if(d2 < r2){
                concatInfo.normal = math.normalize(c2p);
                concatInfo.position = sphere.center +  concatInfo.normal * sphere.radius;
                return true;
            }else{
                return false;
            }
        }

        /// <summary>
        /// 检查点是否在Box内
        /// </summary>
        public static bool PointInside(float3 p,BoxDesc box){
            var o = box.min; //min作为原点
            p = p - o;
            float3 projOnAxis = new float3(
                math.dot(p,box.ax.xyz),
                math.dot(p,box.ay.xyz),
                math.dot(p,box.az.xyz)
            );
            float3 axisLength = new float3(box.ax.w,box.ay.w,box.az.w);
            return math.all(projOnAxis > 0) && math.all(projOnAxis < axisLength);
        }

        /// <summary>
        /// 获取box表面离p最近的点。
        /// 返回值表示p是否在box内部
        /// </summary>
        public static bool GetClosestSurfacePoint(float3 p,BoxDesc box,out ConcatInfo concatInfo){
            var o = box.min; //min作为原点
            p = p - o;
            float3 projOnAxis = new float3(
                math.dot(p,box.ax.xyz),
                math.dot(p,box.ay.xyz),
                math.dot(p,box.az.xyz)
            );
            float3 axisLength = new float3(box.ax.w,box.ay.w,box.az.w);
            float3 side = math.step( axisLength * 0.5f, projOnAxis); // >0.5 => 1 | <0.5 => 0
            float3 signedDist = (1 - side * 2) * (projOnAxis - side * axisLength);
            bool inside = math.all(signedDist > 0);
            if(inside){
                var dst = signedDist.x;
                var axis = box.ax;
                var sideFlag = side.x;
                var axisIndex = 0;
                if(signedDist.y < dst){
                    dst = signedDist.y;
                    axisIndex = 1;
                    sideFlag = side.y;
                    axis = box.ay;
                }
                if(signedDist.z < dst){
                    dst = signedDist.z;
                    sideFlag = side.z;
                    axisIndex = 2;
                    axis = box.az;
                }
                concatInfo = new ConcatInfo();
                concatInfo.normal = sideFlag == 1?axis.xyz:-axis.xyz;
                var offset = (projOnAxis[axisIndex] - sideFlag * axis.w);
                concatInfo.position = o + p - axis.xyz * offset;
                return true;
            }else{
                concatInfo = default;
                return false;
            }


        }


        /// <summary>
        /// 检查点是否在胶囊体内
        /// </summary>
        public static bool PointInside(float3 p ,CapsuleDesc capsule){
            var o = capsule.c0; //c0作为原点
            var radius = capsule.radius;
            var axis = capsule.axis;
            p = p - o;
            var proj = math.dot(p,axis.xyz); //p点在轴上的投影
            if(proj < - radius || proj > axis.w + radius){
                return false;
            }
            var r2 = radius * radius;
            if(proj >=0 && proj <= axis.w){
                //轴上投影在圆柱体之间
                var dist2 = math.dot(p,p) - proj * proj; //计算p到轴的垂直距离平方
                return dist2 < r2;
            }
            if(proj >= -radius && proj < 0){
                //轴上投影处于原点附近
                return math.dot(p,p) < r2;
            }
            if(proj <= axis.w + radius){
                //轴上投影处于另一头附近
                var v = p - (axis.xyz * axis.w);
                return math.dot(v,v) < r2;
            }
            return false;
        }

        public static bool GetClosestSurfacePoint(float3 p,CapsuleDesc capsule,out ConcatInfo concatInfo){
            concatInfo = default(ConcatInfo);
            var o = capsule.c0; //c0作为原点
            var radius = capsule.radius;
            var axis = capsule.axis;
            p = p - o;
            var proj = math.dot(p,axis.xyz); //p点在轴上的投影
            if(proj < - radius || proj > axis.w + radius){
                return false;
            }
            var r2 = radius * radius;
            if(proj >=0 && proj <= axis.w){
                //轴上投影在圆柱体之间
                var dist2 = math.dot(p,p) - proj * proj; //计算p到轴的垂直距离平方
                if(dist2 < r2){
                    var q = axis.xyz * proj;
                    concatInfo.normal = math.normalize(p - q);
                    concatInfo.position =  o + q +  concatInfo.normal * radius;
                    return true;
                }else{
                    return false;
                }
            }
            if(proj >= -radius && proj < 0){
                //轴上投影处于原点附近
                var c2p = p;
                if(math.dot(c2p,c2p) < r2){
                    concatInfo.normal = math.normalize(c2p);
                    concatInfo.position = o + radius * concatInfo.normal;
                    return true;
                }else{
                    return false;
                }
            }
            if(proj <= axis.w + radius){
                //轴上投影处于另一头附近
                var c = (axis.xyz * axis.w);
                var c2p = p - c;
                if(math.dot(c2p,c2p) < r2){
                    concatInfo.normal = math.normalize(c2p);
                    concatInfo.position = o + c + radius * concatInfo.normal;
                    return true;
                }else{
                    return false;
                }
            }
            return false;
        }


    }

    public struct ConcatInfo{
        public float3 position;
        public float3 normal;
    }
}
