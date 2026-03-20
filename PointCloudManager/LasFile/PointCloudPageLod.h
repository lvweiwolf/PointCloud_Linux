//*****************************************************
//
//    @copyright      激光点云组
//    @version        v1.0
//    @file          PointCloudPageLod.h
//    @author         AM
//    @data          2022/4.15
//    @brief        点云PageLod节点
//*****************************************************

#ifndef POINT_CLOUD_PAGELOD_H_
#define POINT_CLOUD_PAGELOD_H_

#include <include/PointCloudManagerExport.h>
#include <include/StringToolkit.h>

#include <osg/PagedLOD>
#include <osgDB/IObjectLoadSaver.h>


class CGeometryNodeVisitor;
class IPageLodAcceptHandler;
class CPointCloudpagedLodSaveLoader;

#define PAGEDLOG_0_MIN_PIXEL_SIZE 1000
#define PAGEDLOG_1_MIN_PIXEL_SIZE 300
#define PAGEDLOG_2_MIN_PIXEL_SIZE 100
#define PAGEDLOG_3_MIN_PIXEL_SIZE 0


#define META_NodeEx(library, name)                                                                 \
	virtual osg::Object* cloneType() const { return new name(); }                                  \
	virtual osg::Object* clone(const osg::CopyOp& copyop) const                                    \
	{                                                                                              \
		return new name(*this, copyop);                                                            \
	}                                                                                              \
	virtual bool isSameKindAs(const osg::Object* obj) const                                        \
	{                                                                                              \
		return dynamic_cast<const name*>(obj) != NULL;                                             \
	}                                                                                              \
	virtual const char* className() const { return #name; }                                        \
	virtual const char* libraryName() const { return #library; }



class POINTCLOUDMANAGER_EXPORT CPointCloudpagedLod : public osg::PagedLOD
{
	friend CPointCloudpagedLodSaveLoader;

public:
	CPointCloudpagedLod(const CPointCloudpagedLod& plod,
						const osg::CopyOp& copyop = osg::CopyOp::DEEP_COPY_ALL);
	CPointCloudpagedLod();
	virtual ~CPointCloudpagedLod();

public:
	/**
	 * 添加子节点（重写以将替换Geometry为SamplesOnOffGeometry）
	 * @return
	 */
	virtual bool addChild(Node* child);
	virtual bool addChild(Node* child, float rmin, float rmax);
	template <class T>
	bool addChild(const osg::ref_ptr<T>& child, float rmin, float rmax)
	{
		if (child == nullptr)
			return false;
		AttachTexCoordArray(child);
		ReplaceSamplesOnOffGeometry(child->asGroup());
		return osg::PagedLOD::addChild(child.get(), rmin, rmax);
	}
	virtual bool addChild(Node* child,
						  float rmin,
						  float rmax,
						  const std::string& filename,
						  float priorityOffset = 0.0f,
						  float priorityScale = 1.0f);
	template <class T>
	bool addChild(const osg::ref_ptr<T>& child,
				  float rmin,
				  float rmax,
				  const std::string& filename,
				  float priorityOffset = 0.0f,
				  float priorityScale = 1.0f)
	{
		if (child == nullptr)
			return false;
		AttachTexCoordArray(child);
		ReplaceSamplesOnOffGeometry(child->asGroup());
		return osg::PagedLOD::addChild(child.get(),
									   rmin,
									   rmax,
									   filename,
									   priorityOffset,
									   priorityScale);
	}


public:
	virtual void traverse(osg::NodeVisitor& nv);

	virtual void accept(osg::NodeVisitor& nv);

	META_NodeEx(osg, CPointCloudpagedLod);

	/**
	 * 获取元素更新状态
	 * @return bool bDirty true:需要更新；false：不需更新
	 */
	bool GetDirty();

	/**
	 * 设置元素更新状态
	 * @param [in] bDirty true:需要更新；false：不需更新
	 * @return
	 */
	virtual void SetDirty(bool bDirty = true);

	/**
	 * 获取元素ID
	 * @return bool bDirty true
	 */
	CString GetIDKey();

	/**
	 * 设置元素ID
	 * @param [in] bDirty true:需要更新；false：不需更新
	 * @return
	 */
	virtual void SetIDKey(CString strID);

	/**
	 *  函数介绍    	获取点数量
	 *
	 *  输入参数    	void
	 *  输出参数
	 *  返回值   	size_t
	 */
	size_t GetPointsNum(void);

	/**
	 *  函数介绍    	设置点数量
	 *
	 *  输入参数    	size_t nPointNum
	 *  输出参数
	 *  返回值   	void
	 */
	void SetPointsNum(size_t nPointNum);

	/**
	 *  @brief    获取模型包围盒
	 *
	 *  @return   osg::BoundingBox
	 */
	const osg::BoundingBox& GetModelBoundingBox() const;

	/**
	 *  @brief    设置模型包围盒
	 *
	 *  @return   osg::BoundingBox
	 */
	void SetModelBoundingBox(const osg::BoundingBox& modelBoundingBox);

	/**
	 *  @brief    获取模型包围盒
	 *
	 *  @return   osg::BoundingBox
	 */
	const osg::BoundingBox& GetRealBoundingBox() const;

	/**
	 *  @brief    设置模型包围盒
	 *
	 *  @return   osg::BoundingBox
	 */
	void SetRealBoundingBox(const osg::BoundingBox& realBoundingBox);

	/**
	 *  @brief    获取是否加载
	 *
	 *  @return   void
	 */
	bool GetIsLoad();

	/*
	 * 函数介绍：获取下一层级的CPointCloudpagedLod节点（不包括读取文件加载到内存中的节点）
	 * 输入参数：void
	 * 输出参数：void
	 * 返回值  ：std::vector<osg::ref_ptr<CPointCloudpagedLod>>
	 */
	std::vector<osg::ref_ptr<CPointCloudpagedLod>> GetNextLevelChidren(void);

	/*
	 * 函数介绍：清理加载的文件节点
	 * 输入参数：void
	 * 输出参数：void
	 * 返回值  ：void
	 */
	void ClearLoadFile(void);

	/*
	 * 函数介绍：清理数据请求对象
	 * 输入参数：void
	 * 输出参数：void
	 * 返回值  ：void
	 */
	void ClearDatabaseRequest(bool bClearChild = true);

	/**
	 *  @brief    设置新的文件名（通常是执行一次手动分类后的名称)
	 *
	 *  @param    std::string name
	 *  @return   void
	 */
	void SetNewFileName(const std::string& name);

	/**
	 *  @brief   获取新的文件名（通常是执行一次手动分类后的名称)
	 *
	 *  @return   std::string
	 */
	std::string GetNewFileName();

	/**
	 *  @brief    设置旧的文件名（通常是执行一次手动分类前的名称)
	 *
	 *  @param    std::string name
	 *  @return   void
	 */
	void SetOldFileName(const std::string& name);

	/**
	 *  @brief   获取旧的文件名（通常是执行一次手动分类前的名称)
	 *
	 *  @return   std::string
	 */
	std::string GetOldFileName();


	/**
	 *  @brief    设置文件链接名
	 *
	 *  @param    unsigned int childNo 子节点索引
	 *  @param    const std::string & filename 文件名
	 *  @return   void
	 */
	void setFileName(unsigned int childNo, const std::string& filename);

	/**
	 * 替换子节点Geometry替换为SamplesOnOffGeometry
	 * @param [in] pGroup
	 * @return
	 */
	void ReplaceSamplesOnOffGeometry(osg::Group* pGroup);

	/**
	 * 附加纹理数组
	 * @param [in] pNode
	 * @return
	 */
	bool AttachTexCoordArray(Node* pNode);

	/** Remove the children from the PagedLOD which haven't been visited since specified expiry time
	 * and expiry frame number. The removed children are added to the removeChildren list passed
	 * into the method, this allows the children to be deleted later at the caller's discretion.
	 * Return true if children are removed, false otherwise. */
	virtual bool removeExpiredChildren(double expiryTime,
									   unsigned int expiryFrame,
									   osg::NodeList& removedChildren) override;

	/**
	 * 锁定节点
	 * @return
	 */
	void LockNode();

	/**
	 * 解锁节点
	 * @return
	 */
	void UnLockNode();

	/**
	 * 绘制图元包围盒
	 * @param [in] nv
	 * @return
	 **/
	void ApplyBoundingBoxGeode(osg::NodeVisitor& nv);

	/**
	 * 设置是否显示包围盒
	 * @param [in] bShow
	 * @return
	 **/
	static void SetShowBoundingBox(int nLevel);

private:
	/**
	 * 纹理加载容错（含容错检查）
	 * @param [in] nv
	 * @param [in] priority
	 * @return
	 */
	void RequestLoadTexture(osg::NodeVisitor& nv, float priority);

	/**
	 * brief:  获取pagedLod层级
	 *
	 * return: int 层级
	 */
	int GetPagedLodLevel();

	/**
	 * brief:  判断是否加载大点云
	 *
	 * return: bool
	 */
	bool JudgeLoadFineBigLas(osg::NodeVisitor& nv);

protected:
	size_t _nPointsNum; // 点数量
	bool _bDirty;
	bool _bIsLoad;
	CString _strId;
	std::string _strNewFileName;
	std::string _strOldFileName;
	osg::BoundingBox _modelBoundingBox;					 // 对应切片的包围盒
	osg::BoundingBox _realBoundingBox;					 // 对应文件的真实包围盒
	osg::ref_ptr<IPageLodAcceptHandler> _pAcceptHandler; // pageLod遍历处理器
	osg::ref_ptr<osg::ProxyNode> _pTexProxyNode;
	osg::ref_ptr<osg::ProxyNode> _pColorProxyNode;
	osg::ref_ptr<osg::Geode> _pBoundingBoxGeode;
};

////////////////////////////////////节点加载保存类
class POINTCLOUDMANAGER_EXPORT CPointCloudpagedLodSaveLoader : public osgDB::IObjectLoadSaver
{
public:
	/**
	 * 节点加载
	 * @param [in] pObject
	 * @param [in] pLoadSave
	 * @return
	 */
	virtual void ObjectLoad(osg::ref_ptr<osg::Object>& pObject, osgDB::IDataStream* pDataStream);

	/**
	 * 节点加载
	 * @param [in] pNode
	 * @param [in] pLoadSave
	 * @return
	 */
	virtual void ObjectSave(osg::ref_ptr<osg::Object> pObject, osgDB::IDataStream* pDataStream);

	/**
	 * 是否处理子节点保存
	 *（平台有节点共享机制，使用osg::group保存逻辑的需要子节点保存，其他不需要处理）
	 * @return
	 */
	virtual bool IsSaveChild();
};

#endif // POINT_CLOUD_PAGELOD_H_
