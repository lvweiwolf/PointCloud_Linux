//////////////////////////////////////////////////////////////////////
// 文件名称：ValueBuffer.h
// 功能描述：内存值缓冲区接口
// 创建标识：吕伟	2022/10/21
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#pragma once
#include <ICloudSegmentation.h>

namespace d3s {
	namespace pcs {

		struct VectorLineString;

		class CRoadVectorizeBuffer : public IValueBuffer
		{
		public:
			CRoadVectorizeBuffer();

			virtual ~CRoadVectorizeBuffer();

			/**
			 *  @brief    获得内存大小字节数
			 *
			 *  @return   size_t
			 */
			virtual size_t GetBytes() const override;


			/**
			 *  @brief    获得内存地址
			 *
			 *  @return   void*
			 */
			virtual void* GetPtr() const override;


			/**
			*  @brief    获得坐标系EPSG
			*
			*  @return   int
			*/
			inline int GetEPSG() const { return _epsg; }

			/**
			 *  @brief    获得道路数据
			 *
			 *  @param    size_t i
			 *
			 *  @return   const d3s::pcs::RoadLineString*
			 */
			const VectorLineString* GetRoad(size_t i) const;

			/**
			 *  @brief    获得道路数量
			 *
			 *  @return   size_t
			 */
			inline size_t GetRoadCount() const { return _roadDS.size(); }

			
			/**
			 *  @brief    从文 *.shp 文件中读取道路矢量数据
			 *
			 *  @param    const std::string & filename
			 *
			 *  @return   bool
			 */
			void ReadShapefile(const std::string& filename);

			/**
			*  @brief    从压缩文件中读取道路矢量数据
			*
			*  @prarm	 const std::string & filename
			*
			*  @return   void
			*/
			void ReadCompressFile(const std::string& filename);

			/**
			 *  @brief    写入到压缩文件
			 *
			 *  @param    const std::string & filename
			 *
			 *  @return   void
			 */
			void WriteCompressFile(const std::string& filename);
				
		private:
			void Clear();

			int _epsg;
			std::vector<VectorLineString*> _roadDS;	// 道路数据集
		};
	}
}
