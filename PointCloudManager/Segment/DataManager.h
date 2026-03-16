/*---------------------------------------------------------------------
*文件名称：CDataManager.h
*功能描述：数据管理器
*创建标识：陶子龙2022.4.29.
*
*修改标识：
*修改描述：
----------------------------------------------------------------------*/
#pragma once

#include "../include/PointCloudManagerExport.h"
	
////////////////////////////////////////////////////////////////////////数据管理器////////////////////////////////////////////////////////////////////////
class POINTCLOUDMANAGER_EXPORT CDataManager
{
public:
	CDataManager(void);
	virtual ~CDataManager(void);


	/*
	* 函数介绍：获取类别转换信息
	* 返回值  ：std::map<int, std::vector<int>>
	*/
	static std::map<int, int> GetTypeConvertInfo();	

};