#include <LasFile/PointCloudToolkit.h>
#include <LasFile/PointCloudToolDefine.h>

#include <Tool/FileToolkit.h>

#include <osgDB/Registry>
#include <osgDB/WriteFile>

const int POINT_CLOUD_MODELNAMELENGTH = 10;
// 文件路径符号
const CString STRING_FILE_PATH_SIGN = L"\\";

CPointCloudToolkit::CPointCloudToolkit(void) {}

CPointCloudToolkit::~CPointCloudToolkit(void) {}

bool CPointCloudToolkit::PtInPolygon(const osg::Vec3d& tagPt,
									 const std::vector<osg::Vec3d>& screenPointVec)
{
	int nCross = 0;
	int nCount = screenPointVec.size();
	for (int i = 0; i < nCount; i++)
	{
		osg::Vec3d p1 = screenPointVec[i];
		osg::Vec3d p2 = screenPointVec[(i + 1) % nCount]; // 点P1与P2形成连线

		if (p1.y() == p2.y())
			continue;
		if (tagPt.y() < std::min<double>(p1.y(), p2.y()))
			continue;
		if (tagPt.y() >= std::max<double>(p1.y(), p2.y()))
			continue;
		// 求交点的x坐标（由直线两点式方程转化而来）

		double x =
			(double)(tagPt.y() - p1.y()) * (double)(p2.x() - p1.x()) / (double)(p2.y() - p1.y()) +
			p1.x();

		// 只统计p1p2与p向右射线的交点
		if (x > tagPt.x())
		{
			nCross++;
		}
	}
	// 交点为偶数，点在多边形之外, 交点为奇数，点在多边形之内
	if ((nCross % 2) == 1)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool CPointCloudToolkit::JudgePolygonInModel(const osg::BoundingBox& boudingBox,
											 const osg::Matrix& vpwMatrix,
											 const std::vector<osg::Vec3d>& selectScreenPointVec)
{
	if (selectScreenPointVec.empty())
		return false;
	// 判断多边形屏幕点是否与模型包络框屏幕点相交，相交则处理文件，反之不处理
	std::vector<osg::Vec3d> pointVec;
	pointVec.push_back(osg::Vec3d(boudingBox.xMin(), boudingBox.yMax(), boudingBox.zMin()));
	pointVec.push_back(osg::Vec3d(boudingBox.xMin(), boudingBox.yMin(), boudingBox.zMin()));
	pointVec.push_back(osg::Vec3d(boudingBox.xMax(), boudingBox.yMin(), boudingBox.zMin()));
	pointVec.push_back(osg::Vec3d(boudingBox.xMax(), boudingBox.yMax(), boudingBox.zMin()));
	pointVec.push_back(osg::Vec3d(boudingBox.xMin(), boudingBox.yMin(), boudingBox.zMax()));
	pointVec.push_back(osg::Vec3d(boudingBox.xMin(), boudingBox.yMax(), boudingBox.zMax()));
	pointVec.push_back(osg::Vec3d(boudingBox.xMax(), boudingBox.yMax(), boudingBox.zMax()));
	pointVec.push_back(osg::Vec3d(boudingBox.xMax(), boudingBox.yMin(), boudingBox.zMax()));
	int nSize = pointVec.size();
	int nMaxX = 0;
	int nMinX = 0;
	int nMaxY = 0;
	int nMinY = 0;
	for (int i = 0; i < nSize; ++i)
	{
		osg::Vec3d point = pointVec.at(i);
		point = point * vpwMatrix;
		// 1.模型投影点一个角在选择点内
		if (PtInPolygon(point, selectScreenPointVec))
			return true;
		if (i == 0)
		{
			nMaxX = ceil(point.x());
			nMinX = ceil(point.x());
			nMaxY = ceil(point.y());
			nMinY = ceil(point.y());
		}
		else
		{
			nMaxX = std::max<int>(nMaxX, ceil(point.x()));
			nMinX = std::min<int>(nMinX, ceil(point.x()));
			nMaxY = std::max<int>(nMaxY, ceil(point.y()));
			nMinY = std::min<int>(nMinY, ceil(point.y()));
		}
	}
	// 2.选择点一个角在模型文件内
	int nScreenMaxX = ceil(selectScreenPointVec.at(0).x());
	int nScreenMinX = ceil(selectScreenPointVec.at(0).x());
	int nScreenMaxY = ceil(selectScreenPointVec.at(0).y());
	int nScreenMinY = ceil(selectScreenPointVec.at(0).y());
	for (auto& selectPoint : selectScreenPointVec)
	{
		int nX = ceil(selectPoint.x());
		int ny = ceil(selectPoint.y());
		nScreenMaxX = std::max<int>(nScreenMaxX, ceil(selectPoint.x()));
		nScreenMinX = std::min<int>(nScreenMinX, ceil(selectPoint.x()));
		nScreenMaxY = std::max<int>(nScreenMaxY, ceil(selectPoint.y()));
		nScreenMinY = std::min<int>(nScreenMinY, ceil(selectPoint.y()));
		if (nX < nMaxX && nX > nMinX && ny > nMinY && ny < nMaxY)
			return true;
	}
	// 3.模型投影点与选择屏幕点“X”字相交
	// 构造模型包络框数组
	std::vector<osg::Vec3d> modelPointVec;
	modelPointVec.push_back(osg::Vec3d(nMinX, nMinY, 0));
	modelPointVec.push_back(osg::Vec3d(nMaxX, nMinY, 0));
	modelPointVec.push_back(osg::Vec3d(nMaxX, nMaxY, 0));
	modelPointVec.push_back(osg::Vec3d(nMinX, nMaxY, 0));
	for (auto& modelPoint : modelPointVec)
	{
		int nX = ceil(modelPoint.x());
		int ny = ceil(modelPoint.y());
		if (nX < nScreenMaxX && nX > nScreenMinX && ny > nScreenMinY && ny < nScreenMaxY)
			return true;
	}

	// 4.模型投影点与选择屏幕点“十”字相交
	if (nScreenMinX < nMinX && nScreenMaxX > nMaxX && nScreenMaxY < nMaxY && nScreenMinY > nMinY)
		return true;
	if (nScreenMinX > nMinX && nScreenMaxX < nMaxX && nScreenMaxY > nMaxY && nScreenMinY < nMinY)
		return true;
	return false;
}

bool CPointCloudToolkit::JudgePolygonInPolygon(const std::vector<osg::Vec3d>& pointVec,
											   const osg::Matrix& vpwMatrix,
											   const std::vector<osg::Vec3d>& selectScreenPointVec)
{
	if (selectScreenPointVec.empty())
		return false;
	// 判断多边形屏幕点是否与模型包络框屏幕点相交，相交则处理文件，反之不处理
	int nSize = pointVec.size();
	for (int i = 0; i < nSize; ++i)
	{
		osg::Vec3d point = pointVec.at(i);
		point = point * vpwMatrix;
		// 1.模型投影点一个角在选择点内
		if (PtInPolygon(point, selectScreenPointVec))
			return true;
	}

	return false;
}

osg::ref_ptr<osg::Node> CPointCloudToolkit::ReadNode(const std::string& pointFilePath,
													 const std::string& texFileName)
{
	osgDB::ReaderWriter::ReadResult rsPointData =
		osgDB::Registry::instance()->readNode(pointFilePath,
											  osgDB::Registry::instance()->getOptions(),
											  false);
	if (!rsPointData.validNode())
	{
		d3s::CLog::Error("CPointCloudToolkit::ReadNode节点信息加载出错!（%s）",
						 pointFilePath.c_str());
		return nullptr;
	}

	osg::ref_ptr<osg::Node> pReturnNode = rsPointData.getNode();
	CGeometryNodeVisitor geoVisitor;
	pReturnNode->accept(geoVisitor);

	// 加载点颜色信息
	{
		std::string strColorFileName = pointFilePath;
		strColorFileName.replace(strColorFileName.find(pc::data::strFilExt),
								 strColorFileName.length(),
								 pc::data::ColorPostStr + pc::data::strFilExt);
		osgDB::ReaderWriter::ReadResult rsColorData =
			osgDB::Registry::instance()->readNode(strColorFileName,
												  osgDB::Registry::instance()->getOptions(),
												  false);
		if (!rsColorData.validNode())
		{
			d3s::CLog::Error("CPointCloudToolkit::ReadNode节点颜色信息加载出错!（%s）",
							 strColorFileName.c_str());
			return pReturnNode;
		}

		CGeometryNodeVisitor colorVisitor;
		rsColorData.getNode()->accept(colorVisitor);
		if (colorVisitor._vecGeo.empty() ||
			(colorVisitor._vecGeo.size() != geoVisitor._vecGeo.size()))
			return pReturnNode;

		for (size_t nIndex = 0; nIndex < geoVisitor._vecGeo.size(); ++nIndex)
		{
			geoVisitor._vecGeo.at(nIndex)->setColorArray(
				colorVisitor._vecGeo.at(nIndex)->getColorArray());
		}
	}

	// 加载纹理数组信息
	std::string strTexFileName = texFileName;
	// strTexFileName.replace(strTexFileName.find(pc::data::strFilExt),
	// 					   strTexFileName.length(),
	// 					   pc::data::TexPostStr + pc::data::strFilExt);
	osgDB::ReaderWriter::ReadResult rsTexData =
		osgDB::Registry::instance()->readNode(strTexFileName,
											  osgDB::Registry::instance()->getOptions(),
											  false);
	if (!rsTexData.validNode())
	{
		d3s::CLog::Error(L"CPointCloudToolkit::ReadNode节点纹理信息加载出错!（%s）",
						 strTexFileName.c_str());
		return pReturnNode;
	}

	CGeometryNodeVisitor texVisitor;
	rsTexData.getNode()->accept(texVisitor);
	if (texVisitor._vecGeo.empty())
		return pReturnNode;

	osg::ref_ptr<osg::Geometry> pTexGeo = texVisitor._vecGeo.at(0)->asGeometry();
	osg::Geometry::ArrayList& texArrList = pTexGeo->getTexCoordArrayList();
	if (geoVisitor._vecGeo.size() != texArrList.size())
		return pReturnNode;

	for (size_t nIndex = 0; nIndex < geoVisitor._vecGeo.size(); ++nIndex)
	{
		geoVisitor._vecGeo.at(nIndex)->setTexCoordArray(0, texArrList.at(nIndex));
	}
	return pReturnNode;
}

bool CPointCloudToolkit::WriteNode(osg::ref_ptr<osg::Node> pNode,
								   const std::string& fileName,
								   EWriteType eType)
{
	try
	{
		if (nullptr == pNode)
			return false;

		CGeometryNodeVisitor geoVisitor;
		pNode->accept(geoVisitor);

		if (geoVisitor._vecGeo.empty())
			return false; // 节点下不存在geometry

		// 收集Geometry中的颜色及纹理信息
		osg::Geometry::ArrayList texArrList;
		osg::Geometry::ArrayList colorArrList;
		for (auto iterGeo : geoVisitor._vecGeo)
		{
			if (1 != iterGeo->getNumTexCoordArrays())
			{
				ASSERT(FALSE && L"写入节点纹理数组数量错误！");
				continue;
			}

			auto& tempArrList = iterGeo->getTexCoordArrayList();
			auto tempColorArray = iterGeo->getColorArray();
			texArrList.insert(texArrList.end(), tempArrList.begin(), tempArrList.end());
			colorArrList.emplace_back(tempColorArray);
			if (eType & EWriteType::ePointData)
			{
				// 将纹理信息分离
				tempArrList.clear();
				iterGeo->setColorArray(nullptr);
			}
		}

		// 写入 点信息 文件
		if (eType & EWriteType::ePointData)
		{
			osgDB::writeNodeFile(*pNode, fileName, &pc::data::S_IVE_OPTION);

			// 写入点颜色文件
			std::string strColorFileName = fileName;
			osg::ref_ptr<osg::Geode> pTempGeode = new osg::Geode;
			for (auto iter : colorArrList)
			{
				osg::ref_ptr<osg::Geometry> pGeometry = new osg::Geometry;
				pGeometry->setColorArray(iter);
				pTempGeode->addChild(pGeometry);
			}
			strColorFileName.replace(strColorFileName.find(pc::data::strFilExt),
									 strColorFileName.length(),
									 pc::data::ColorPostStr + pc::data::strFilExt);
			osgDB::writeNodeFile(*pTempGeode,
								 strColorFileName,
								 osgDB::Registry::instance()->getOptions());

			// 还原颜色及纹理信息
			size_t geoCount = geoVisitor._vecGeo.size();
			ASSERT(geoCount == texArrList.size() && L"geometry与纹理数量不对应！");
			ASSERT(geoCount == colorArrList.size() && L"geometry与颜色数量不对应！");

			if (geoCount == texArrList.size() && geoCount == colorArrList.size())
			{
				for (size_t nIndex = 0; nIndex < geoCount; ++nIndex)
				{
					geoVisitor._vecGeo.at(nIndex)->setTexCoordArray(0, texArrList.at(nIndex));
					geoVisitor._vecGeo.at(nIndex)->setColorArray(colorArrList.at(nIndex));
				}
			}
		}

		// 写入 纹理数组信息
		if (eType & EWriteType::eTexCoord)
		{
			std::string strTexFileName = fileName;
			osg::ref_ptr<osg::Geode> pTempGeode = new osg::Geode;
			osg::ref_ptr<osg::Geometry> pGeometry = new osg::Geometry;
			pGeometry->setTexCoordArrayList(texArrList);
			pTempGeode->addChild(pGeometry);
			strTexFileName.replace(strTexFileName.find(pc::data::strFilExt),
								   strTexFileName.length(),
								   pc::data::TexPostStr + pc::data::strFilExt);
			CString sTexFile = CStringToolkit::UTF8ToCString(strTexFileName);
			if (CFileToolkit::FileExist(sTexFile) && strTexFileName.find(pc::data::strFilExt) > 0)
			{
				CFileToolkit::DeleteToRecycle(sTexFile);
			}
			osgDB::writeNodeFile(*pTempGeode,
								 strTexFileName,
								 osgDB::Registry::instance()->getOptions());
		}

		return true;
	}
	catch (...)
	{
		d3s::CLog::Error("CPointCloudToolkit::WriteNode发生异常，函数参数：pNode==nullptr(%d)"
						 "、fileName(%s)、eType(%d)",
						 (nullptr == pNode),
						 fileName.c_str(),
						 eType);
	}
	return false;
}

osg::ref_ptr<osg::Geode> CPointCloudToolkit::BuildBoundingBoxGeode(osg::BoundingBox& box,
																   osg::Vec4& color)
{
	auto pGeode = new osg::Geode;
	auto pGeometry = new osg::Geometry;
	pGeode->addChild(pGeometry);
	auto pVertArray = new osg::Vec3Array;
	pGeometry->setVertexArray(pVertArray);
	osg::ref_ptr<osg::Vec4Array> blackColors = new osg::Vec4Array;
	blackColors->push_back(color);
	pGeometry->setColorArray(blackColors);
	pGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);

	osg::Vec3 c000(osg::Vec3(box.xMin(), box.yMin(), box.zMax()));
	osg::Vec3 c100(osg::Vec3(box.xMax(), box.yMin(), box.zMax()));
	osg::Vec3 c110(osg::Vec3(box.xMax(), box.yMax(), box.zMax()));
	osg::Vec3 c010(osg::Vec3(box.xMin(), box.yMax(), box.zMax()));

	osg::Vec3 c001(osg::Vec3(box.xMin(), box.yMin(), box.zMin()));
	osg::Vec3 c101(osg::Vec3(box.xMax(), box.yMin(), box.zMin()));
	osg::Vec3 c111(osg::Vec3(box.xMax(), box.yMax(), box.zMin()));
	osg::Vec3 c011(osg::Vec3(box.xMin(), box.yMax(), box.zMin()));

	pVertArray->push_back(c000);
	pVertArray->push_back(c100);
	pVertArray->push_back(c110);
	pVertArray->push_back(c010);
	pGeometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP, 0, 4));
	pVertArray->push_back(c001);
	pVertArray->push_back(c011);
	pVertArray->push_back(c111);
	pVertArray->push_back(c101);
	pGeometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP, 4, 4));
	pVertArray->push_back(c000);
	pVertArray->push_back(c001);
	pVertArray->push_back(c100);
	pVertArray->push_back(c101);
	pVertArray->push_back(c110);
	pVertArray->push_back(c111);
	pVertArray->push_back(c010);
	pVertArray->push_back(c011);
	pGeometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 8, 8));
	return pGeode;
}


CString CPointCloudToolkit::GetNewPageFileName(CString strDatabasePath, const CString& strFileName)
{
	strDatabasePath.TrimRight(L"/");
	strDatabasePath.TrimRight(L"\\");
	strDatabasePath.Format(_T("%s\\"), (LPCWSTR)strDatabasePath);
	if (!CFileToolkit::DirectoryExist(strDatabasePath))
		return L"";

	CString strBaseName;
	CString strExtendName = CFileToolkit::GetFileExtendName(strFileName);
	CString strNewFileName = strFileName;
	bool bOut = false;
	while (!bOut)
	{
		strBaseName = CFileToolkit::GetFileBaseName(strNewFileName);
		int nIndex = strBaseName.Find(L"&");
		if (nIndex < 0)
			strNewFileName.Format(L"%s&%d%s", (LPCWSTR)strBaseName, 1, (LPCWSTR)strExtendName);
		else
		{
			CString strNum = strBaseName.Right(strBaseName.GetLength() - nIndex - 1);
			CString strName = strBaseName.Left(nIndex);
			int Nnum = CStringToolkit::StrToInt(strNum) + 1;
			strNewFileName.Format(L"%s&%d%s", (LPCWSTR)strName, Nnum, (LPCWSTR)strExtendName);
		}
		bOut = !CFileToolkit::FileExist(strDatabasePath + strNewFileName);
	}
	return strNewFileName;
}
