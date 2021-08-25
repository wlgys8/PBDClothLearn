using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;
using Unity.Mathematics;

namespace PBDLearn
{
    public class ClothRenderer : MonoBehaviour
    {
        public WindZone windZone;
        private ClothSimulator _simulator;

        private Mesh _mesh;
        private MeshFilter _meshFilter;

        public UnityColliderProxy[] colliderProxies;

        public ClothSetting setting;

        public bool drawGizmos;

        private ClothMeshModifier _meshModifier;

        private Dictionary<int,Transform> _attachments = new Dictionary<int, Transform>();

        void Awake()
        {
            _meshFilter = GetComponent<MeshFilter>();
            _meshFilter.sharedMesh = Object.Instantiate(_meshFilter.sharedMesh);
            _mesh = _meshFilter.sharedMesh;
            _meshModifier = ClothMeshModifier.CreateFromMeshFilter(_meshFilter);
            _simulator = new ClothSimulator(_meshModifier,this.setting);
            foreach(var c in colliderProxies){
                _simulator.AddCollider(c);
            }
        }

        void Start(){
            GetComponent<MeshRenderer>().sharedMaterial.EnableKeyword("_INPUT_WORLD_VERTEX");
            transform.localPosition = Vector3.zero;
        }

        public void Attach(int vertexIndex,Transform target){
            if(!target){
                throw new System.Exception("target can not be null");
            }
            _attachments.Add(vertexIndex,target);
            target.position = _simulator.positions[vertexIndex];
        }

        void OnDestroy(){
            if(_simulator != null){
                _simulator.Dispose();
                _simulator = null;
            }
             GetComponent<MeshRenderer>().sharedMaterial.DisableKeyword("_INPUT_WORLD_VERTEX");
        }

        void Update()
        {
            if(_simulator != null){
                foreach(var pair in _attachments){
                    _simulator.AddPinConstraint(pair.Key,pair.Value.position);
                }
                _simulator.SetFieldForce(windZone.transform.forward * windZone.windMain);
                _simulator.Step(Time.deltaTime);
            }
        }

        void LateUpdate(){
            if(_simulator != null){
                _simulator.ComputeAsyncJobs();
                _mesh.SetVertices(_simulator.positions);
                _mesh.SetNormals(_simulator.normals);
                _mesh.RecalculateBounds();
            }
        }

        void OnDrawGizmos(){
            if(drawGizmos){
                if(_simulator != null){
                    foreach(var p in _simulator.positions){
                        Gizmos.DrawSphere(p,0.05f);
                    }
                    for(var i = 0; i < _simulator.distanceConstraintCount;i ++){
                        var info = _simulator.GetDistanceConstraintInfo(i);
                        var fromPos = _simulator.positions[info.vIndex0];
                        var toPos = _simulator.positions[info.vIndex1];
                        Gizmos.DrawLine(fromPos,toPos);
                    }
                }
            }
        }
    }
}
