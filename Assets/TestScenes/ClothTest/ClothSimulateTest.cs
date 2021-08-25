using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using PBDLearn;

public class ClothSimulateTest : MonoBehaviour
{

    public ClothRenderer cloth;

    public List<AttachBind> attachBinds = new List<AttachBind>();

    void Start(){
        foreach(var a in attachBinds){
            cloth.Attach(a.vertexIndex,a.target);
        }
    }



    [System.Serializable]
    public class AttachBind{
        public int vertexIndex;
        public Transform target;
    }
    

}
