using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using PBDLearn;

public class IntersectionTestScene : MonoBehaviour
{

    public UnitySphereColliderProxy sphereColliderProxy;
    public UnityBoxColliderProxy boxColliderProxy;

    public UnityCapsuleColliderProxy capsuleColliderProxy;

    private bool _inside = false;

    private UnityColliderProxy[] _colliderProxies;

    void Start()
    {
        _colliderProxies = new UnityColliderProxy[]{
            boxColliderProxy,
            sphereColliderProxy,
            capsuleColliderProxy
        };
    }

    private ConcatInfo _concatInfo;
    void Update()
    {
        var pos = transform.position;
        foreach(var c in _colliderProxies){
            _inside = c.GetClosestSurfacePoint(pos,out _concatInfo);
            if(_inside){
                break;
            }
        }
    }

    void OnDrawGizmos(){
        var originalColor = Gizmos.color;
        if(_inside){
            Gizmos.DrawSphere(_concatInfo.position,0.1f);
            Gizmos.DrawRay(_concatInfo.position,_concatInfo.normal);
            Gizmos.color = Color.red;
        }
        Gizmos.DrawSphere(this.transform.position,0.1f);
        Gizmos.color = originalColor;
    }
}
