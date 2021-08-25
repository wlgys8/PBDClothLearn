using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace PBDLearn
{

    [System.Serializable]
    public class ClothSetting
    {

        /// <summary>
        /// 布料密度，单位kg/m^2
        /// </summary>
        [Range(0.01f,1)]
        [SerializeField]
        private float _density = 1;

        /// <summary>
        /// 约束求解迭代次数，必须为2的倍数
        /// </summary>
        [Range(1,10)]
        [SerializeField]
        private int _constraintSolverIteratorCount = 2;

        /// <summary>
        /// 压缩约束系数
        /// </summary>
        [Range(0.01f,1f)]
        [SerializeField]
        private float _compressStiffness = 0.8f;

        /// <summary>
        /// 拉伸约束系数
        /// </summary>
        [Range(0.01f,1f)]
        [SerializeField]
        private float _stretchStiffness = 0.8f;

        /// <summary>
        /// 弯曲约束系数
        /// </summary>
        [Range(0.01f,1)]
        [SerializeField]
        private float _bendStiffness = 0.1f;

        /// <summary>
        /// 阻尼系数
        /// </summary>
        [Range(0.01f,10)]
        [SerializeField]

        private float _damper = 1;


        public float density{
            get{
                return _density;
            }
        }

        public int constraintSolverIteratorCount{
            get{
                return _constraintSolverIteratorCount;
            }
        }

        public float compressStiffness{
            get{
                return this._compressStiffness;
            }
        }

        public float stretchStiffness{
            get{
                return _stretchStiffness;
            }
        }

        public float bendStiffness{
            get{
                return _bendStiffness;
            }
        }

        public float damper{
            get{
                return _damper;
            }
        }


    }
}
