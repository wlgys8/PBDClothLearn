using UnityEngine;
using Unity.Mathematics;

namespace PBDLearn
{
    [RequireComponent(typeof(SphereCollider))]
    public class UnitySphereColliderProxy : UnityColliderProxy
    {
        private SphereDesc _desc;
        private SphereCollider _collider;
        private int _lastUpdateFrame;

        private SphereCollider unityCollider{
            get{
                if(!_collider){
                    _collider = GetComponent<SphereCollider>();
                }
                return _collider;
            }
        }

        public SphereDesc desc{
            get{
                if(_lastUpdateFrame == Time.frameCount){
                    return _desc;
                }
                _lastUpdateFrame = Time.frameCount;
                _desc.center = transform.position;
                _desc.radius = unityCollider.radius * transform.localScale.x;
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
            collidersGroup.AddSphere(this.desc,rigidbodyDesc,this.entityId);
        }

    }
}
