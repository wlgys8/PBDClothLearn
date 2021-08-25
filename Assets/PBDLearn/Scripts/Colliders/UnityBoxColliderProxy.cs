using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace PBDLearn
{
    [RequireComponent(typeof(BoxCollider))]
    public class UnityBoxColliderProxy : UnityColliderProxy
    {
        private BoxDesc _desc;

        private BoxCollider _collider;

        private int _lastUpdateFrame;

        private BoxCollider unityCollider{
            get{
                if(!_collider){
                    _collider = GetComponent<BoxCollider>();
                }
                return _collider;
            }
        }

        public BoxDesc desc{
            get{
                if(Time.frameCount == _lastUpdateFrame){
                    return _desc;
                }
                _lastUpdateFrame = Time.frameCount;
                var size = unityCollider.size;
                var min = unityCollider.center - size * 0.5f;
                var localToWorld = unityCollider.transform.localToWorldMatrix;
                _desc.min = localToWorld.MultiplyPoint(min);
                var dx = localToWorld.MultiplyVector(new Vector3(size.x,0,0));
                var dy = localToWorld.MultiplyVector(new Vector3(0,size.y,0));
                var dz = localToWorld.MultiplyVector(new Vector3(0,0,size.z));
                _desc.ax = new float4(dx.normalized,dx.magnitude);
                _desc.ay = new float4(dy.normalized,dy.magnitude);
                _desc.az = new float4(dz.normalized,dz.magnitude);
                return _desc;
            }
        }

        public override bool PointInside(float3 p)
        {
            return IntersectUtil.PointInside(p,desc);
        }
        public override bool GetClosestSurfacePoint(float3 p, out ConcatInfo concatInfo)
        {
            return IntersectUtil.GetClosestSurfacePoint(p,desc,out concatInfo);
        }

        public override void FillColliderDesc(CollidersGroup collidersGroup)
        {
            RigidbodyDesc rigidbodyDesc = default(RigidbodyDesc);
            float mass = 0;
            if(_rigidbody && !_rigidbody.isKinematic){
                mass = _rigidbody.mass;
                rigidbodyDesc.mass = _rigidbody.mass;
                rigidbodyDesc.velocity = _rigidbody.velocity;
            }
            collidersGroup.AddBox(this.desc,rigidbodyDesc,this.entityId);
        }

    
    }
}
