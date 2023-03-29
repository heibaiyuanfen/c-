using ScaraVisionDll.Camera;
using HalconDotNet;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Security.Policy;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Xml.Linq;
using VisionControls;
using ScaraVisionDll.Models;
using VisionControls.Win32;
using System.Drawing.Printing;
using ViewWindow.Model;
using ScaraVisionDll.Service;

namespace ScaraVisionDll.Arithmetic
{
    public class VisionArithmetic
    {
        private string ConfigPath;
        private CamProject _CamProject;
        private HHomMat2D TransHHomMat2D;
        public event Action< FindResult> EventFindRunResult;//定义事件 传出结果
        public VisionArithmetic(CamProject camProject)
        {
            _CamProject = camProject;
            TransHHomMat2D = _CamProject._hHomMat2D;
            ConfigPath = AppDomain.CurrentDomain.BaseDirectory + _CamProject.SavePath;
        }
        public List<CailbCoordinate> FindCircle(HImage _Himage,HRegion RoiRegion)
        {
            List<CailbCoordinate> result = new List<CailbCoordinate>();   
            _Himage = _Himage.Rgb1ToGray();//转灰度图
            _Himage= _Himage.ReduceDomain(RoiRegion);
            //筛选出里面小圆
           var ConnectionCirRegion = _Himage.BinaryThreshold("max_separability", "dark", out HTuple UsedThreshold1).Connection();
            //var SelectRegion=   ConnectionCirRegion.SelectShape((new HTuple("circularity")).TupleConcat("area"), "and", (new HTuple(0.7936)).TupleConcat(5963.3), (new HTuple(1)).TupleConcat(40825.7));
            var SelectRegion = ConnectionCirRegion.SelectShape((new HTuple("circularity")).TupleConcat("area"), "and", (new HTuple(0.7936)).TupleConcat(2600), (new HTuple(1)).TupleConcat(2800));
            SelectRegion.AreaCenter(out HTuple hv_Row, out HTuple hv_Col);
            sort_pairs(hv_Row, hv_Col, "1", out HTuple NewRow, out HTuple NewCol);
          var NewRowList=  NewRow.ToDArr().ToList();
          var NewColList = NewCol.ToDArr().ToList();
            for (int i = 0; i < NewRowList.Count; i++)
            {
                CailbCoordinate coordinate = new CailbCoordinate()
                {
                    Image_Row = Convert.ToDouble(NewRowList[i].ToString("0.0")),
                    Image_Col = Convert.ToDouble(NewColList[i].ToString("0.0")),
                    ID = i + 1
                    
                };
                result.Add(coordinate);
            }
            return result;  
        }
        private void sort_pairs(HTuple hv_T1, HTuple hv_T2, HTuple hv_SortMode, out HTuple hv_Sorted1,out HTuple hv_Sorted2)
        {
            // Local iconic variables 

            // Local control variables 

            HTuple hv_Indices1 = new HTuple(), hv_Indices2 = new HTuple();
            // Initialize local and output iconic variables 
            hv_Sorted1 = new HTuple();
            hv_Sorted2 = new HTuple();
            //Sort tuple pairs.
            //
            //input parameters:
            //T1: first tuple
            //T2: second tuple
            //SortMode: if set to '1', sort by the first tuple,
            //   if set to '2', sort by the second tuple
            //
            if ((int)((new HTuple(hv_SortMode.TupleEqual("1"))).TupleOr(new HTuple(hv_SortMode.TupleEqual(
                1)))) != 0)
            {
                hv_Indices1.Dispose();
                HOperatorSet.TupleSortIndex(hv_T1, out hv_Indices1);
                hv_Sorted1.Dispose();
                using (HDevDisposeHelper dh = new HDevDisposeHelper())
                {
                    hv_Sorted1 = hv_T1.TupleSelect(
                        hv_Indices1);
                }
                hv_Sorted2.Dispose();
                using (HDevDisposeHelper dh = new HDevDisposeHelper())
                {
                    hv_Sorted2 = hv_T2.TupleSelect(
                        hv_Indices1);
                }
            }
            else if ((int)((new HTuple((new HTuple(hv_SortMode.TupleEqual("column"))).TupleOr(
                new HTuple(hv_SortMode.TupleEqual("2"))))).TupleOr(new HTuple(hv_SortMode.TupleEqual(
                2)))) != 0)
            {
                hv_Indices2.Dispose();
                HOperatorSet.TupleSortIndex(hv_T2, out hv_Indices2);
                hv_Sorted1.Dispose();
                using (HDevDisposeHelper dh = new HDevDisposeHelper())
                {
                    hv_Sorted1 = hv_T1.TupleSelect(
                        hv_Indices2);
                }
                hv_Sorted2.Dispose();
                using (HDevDisposeHelper dh = new HDevDisposeHelper())
                {
                    hv_Sorted2 = hv_T2.TupleSelect(
                        hv_Indices2);
                }
            }

            hv_Indices1.Dispose();
            hv_Indices2.Dispose();

            return;
        }
        public void CreatShapeModel(HImage _HModelImage,VisionInfo visionInfo, List<ROI> regions,string color,string SavePath)
        {
            if (regions.Count <= 0)
            {
                MessageBox.Show("请框选目标区域后生成模板");
                return ;
            }
            //将框与抓取点的ROI取出
            var Model_hRegion= regions.Where(x => x.Color == color).FirstOrDefault().getRegion();
            //先把识别区域提取出来
            var ModelRecognitionImage = _HModelImage.ReduceDomain(Model_hRegion);
            //创建模板
            visionInfo.HShapeModel.CreateShapeModel(ModelRecognitionImage, 0, (new HTuple(0)).TupleRad(), (new HTuple(360)).TupleRad(),
                "auto", "none", "use_polarity", visionInfo.CreateContrast, visionInfo.CreateMinContrast);
            //保存为模板信息
            visionInfo._TemplateInfo = FindSingleShapeModel(_HModelImage, visionInfo);
            SaveModel(visionInfo.HShapeModel, SavePath);

        }
        private WorkpieceInfo FindSingleShapeModel(HImage _HImage,VisionInfo visionInfo)
        {
            WorkpieceInfo _FindResult = new WorkpieceInfo();
            if (_HImage == null || !_HImage.IsInitialized())
            {
                Logger.Error($"{visionInfo.VisionName}:FindShapeModel: hImage == null || !hImage.IsInitialized() ");
                return null;
                
            }
            if(!visionInfo.HShapeModel.IsInitialized()|| visionInfo.HShapeModel == null)
            {
                MessageBox.Show(visionInfo.VisionName+":模板异常");
            }
            {
                visionInfo.HShapeModel.FindShapeModel(_HImage,
                    (new HTuple(0)).TupleRad(),
                    (new HTuple(360)).TupleRad(),
                    visionInfo.FindMinScore,
                  1,
                   visionInfo.FindMaxOverlap,
                     "least_squares",
                     0,
                     visionInfo.FindGreediness,
                      out HTuple Row, out HTuple Col, out HTuple Angle, out HTuple Score);
                //判断是否为空
                if (Row.ToSArr().Any(x => !string.IsNullOrEmpty(x)))
                {
                    _FindResult.ModelRow = Row.D;
                    _FindResult.ModelCol= Col.D;
                    _FindResult.ModelAngle= Angle.D;
                    _FindResult.ModelScore = Score.D;
                }
            }
            return _FindResult;

        }
        public List<WorkpieceInfo> FindShapeModel(HImage _HImage, VisionInfo visionInfo)
        {
            List<WorkpieceInfo> _ListWorkpieceResult = new List<WorkpieceInfo>();
            if (_HImage == null || !_HImage.IsInitialized())
            {
                Logger.Error($"FindShapeModel: hImage == null || !hImage.IsInitialized() ");
                return null;
            }
            Stopwatch stopwatch = new Stopwatch();
            stopwatch.Restart();
           
                visionInfo.HShapeModel.FindShapeModel(_HImage, 
                    (new HTuple(0)).TupleRad(),
                    (new HTuple(360)).TupleRad(),
                    visionInfo.FindMinScore,
                  visionInfo.FindNumMatches,
                   visionInfo.FindMaxOverlap,
                     "least_squares",
                     0,
                     visionInfo.FindGreediness,
                      out HTuple Row, out HTuple Col, out HTuple Angle, out HTuple Score);
                //判断是否为空
                if (Row.ToSArr().Any(x => !string.IsNullOrEmpty(x)))
                {
                    for (int i = 0; i < Score.DArr.Length; i++)
                    {
                    _ListWorkpieceResult.Add(new WorkpieceInfo()
                        {
                            ModelRow = Row.TupleSelect(i).D,
                            ModelCol = Col.TupleSelect(i).D,
                            ModelAngle = Angle.TupleSelect(i).D,
                             ModelScore = Score.DArr[i]
                        }) ;
                    }
                }
            return _ListWorkpieceResult;
        }
        public FindResult processed(HImage _HImag,List<WorkpieceInfo> WorkpieceInfos,List< VisionInfo > VisionInfos,List<ROI> regions)
        {
            FindResult findResult = new FindResult();
            WorkpieceInfo workpieceInfo = new WorkpieceInfo();
            if (WorkpieceInfos.Count>0)
            {
                var BigVision= VisionInfos.Where(x => x.VisionName.Contains("Big")).FirstOrDefault();
                var SmallVision = VisionInfos.Where(x => x.VisionName.Contains("Small")).FirstOrDefault();
                List<HObject> ListHobj = new List<HObject>();
               // Hobj.GenEmptyObj();
                foreach (var Workpiece in WorkpieceInfos)
                {
                    if (findResult.FindCoordinateList.Count >= _CamProject.FindCount) break;
                    if(_CamProject.IsUserOpRecognite)//使用正反识别
                    {
                        //1 先仿射变换出到标准位置,并变换图像
                        HHomMat2D HHomMat2D1 = new HHomMat2D();
                        HHomMat2D1.VectorAngleToRigid(Workpiece.ModelRow, Workpiece.ModelCol, Workpiece.ModelAngle,
                          BigVision._TemplateInfo.ModelRow, BigVision._TemplateInfo.ModelCol, BigVision._TemplateInfo.ModelAngle);
                        var AffineImage = HHomMat2D1.AffineTransImage(_HImag, "constant", "false");
                        //2 生成矩形并截取下来，并用小模板比对
                        var RecRegion = regions.Where(x => x.Color == "orange").FirstOrDefault().getRegion();
                        var RecRedImage = AffineImage.ReduceDomain(RecRegion);
                        //3 判断小模板是否存在 存在就是反面 反之就是正面
                        workpieceInfo = FindSingleShapeModel(RecRedImage, SmallVision);
                    }
                    if (!_CamProject.IsUserOpRecognite
                        ||(_CamProject.IsUserOpRecognite&&workpieceInfo.ModelRow==0&& workpieceInfo!=null))
                       // ||( _CamProject.IsUserOpRecognite && workpieceInfo.ModelRow != 0 && workpieceInfo != null&& !_CamProject.IsReverseLogic))
                    {
                        //4 生成抓取点的矩形，并生成抓取点
                        double CapRow = regions.Where(x => x.Color == "red").FirstOrDefault().getModelData()[0];
                        double CapCol = regions.Where(x => x.Color == "red").FirstOrDefault().getModelData()[1];
                        var CapRegion  = regions.Where(x => x.Color == "red").FirstOrDefault().getRegion();
                        HHomMat2D HHomMat2D2 = new HHomMat2D();
                        HHomMat2D2.VectorAngleToRigid(BigVision._TemplateInfo.ModelRow, BigVision._TemplateInfo.ModelCol, BigVision._TemplateInfo.ModelAngle,
                        Workpiece.ModelRow, Workpiece.ModelCol, Workpiece.ModelAngle);
                        var AffineRow= HHomMat2D2.AffineTransPoint2d(CapRow, CapCol, out double AffineCol);
                        //矩形
                        var AffineCapRegion= HHomMat2D2.AffineTransRegion(CapRegion, "nearest_neighbor"); //仿射变换后的矩形
                        var AffineCapRegionArea = _HImag.ReduceDomain(AffineCapRegion).Threshold(0, 180.0).AreaCenter(out double AreaCenterx, out double AreaCentery);
                        //var numDate= (AffineCapRegionArea/_CamProject.CaptureArea ) *100;
                        //*if(AffineCapRegionArea> _CamProject.ErrPercentArea)
                            //*continue;
                        //5.识别遮挡物（沾污)
                        var Shelterobj = regions.Where(x => x.Color == "yellow").FirstOrDefault();
                        if(Shelterobj != null)
                        {
                            var ShelterRegion = Shelterobj.getRegion();
                            var AffineShelterRegion = HHomMat2D2.AffineTransRegion(ShelterRegion, "nearest_neighbor"); //仿射变换后的矩形
                            var AffineShelterRegionArea = _HImag.ReduceDomain(AffineShelterRegion).Threshold(0, 180.0).AreaCenter(out double ShelterAreaCenterx, out double ShelterAreaCentery);
                            if (AffineShelterRegionArea > _CamProject.ErrPercentArea)
                                continue;
                        }
                        HXLDCont hXLDCont = new HXLDCont();
                        hXLDCont.GenCrossContourXld(AffineRow, AffineCol, 20, Workpiece.ModelAngle);
                        
                        //实际转化的点==实际坐标
                        var X = _CamProject._hHomMat2D.AffineTransPoint2d(AffineRow, AffineCol, out double Y);
                        //5 仿射变换轮廓用于显示
                        HHomMat2D HHomMat2D3 = new HHomMat2D();
                       var Contour = BigVision.HShapeModel.GetShapeModelContours(1);
                        HHomMat2D3.VectorAngleToRigid(0.0, 0, 0, Workpiece.ModelRow, Workpiece.ModelCol, Workpiece.ModelAngle);
                       var AffineContour= HHomMat2D3.AffineTransContourXld(Contour);
                        HObject Hobj = new HObject();
                        Hobj.GenEmptyObj();
                         Hobj = Hobj.ConcatObj(AffineContour);
                         Hobj = Hobj.ConcatObj(hXLDCont);
                        {
                            //传递角度
                            HOperatorSet.TupleDeg(Workpiece.ModelAngle, out HTuple Deg);
                            findResult.FindCoordinateList.Add(new ResultCoord()
                            {
                                Image_Row = AffineRow,
                                Image_Col = AffineCol,
                                Scara_Row = X,
                                Scara_Col = Y,
                                Angle= Deg.D
                            });
                            //传递显示
                            findResult._ListHobj.Add(Hobj);
                            //传递识别分数
                            findResult.Find_ListScore.Add(Workpiece.ModelScore);
                        }

                    }
                    else
                    {
                     
                    }
                }
            }
            findResult._Himage = _HImag;
            return findResult;
        }
        public void SaveModel(HShapeModel hShapeModel,string SavePath)
        {
            hShapeModel.WriteShapeModel(SavePath);
            Logger.Info("保存模板成功");
        }
        private int CheckExistCount(HImage _Himge)
        {
         var ConnRegions= _Himge.BinaryThreshold("max_separability" , "dark", out HTuple UsedThreshold).Connection().ErosionRectangle1(30,30);
            var res= ConnRegions.SelectShape("area", "and", 1000, 400000);
            HOperatorSet.CountObj(res, out HTuple Number);
            return Number.I;
        }

        public void RunArithmetic(HImage hImage)
        {
            Stopwatch st = new Stopwatch();
            st.Restart();
            var BigInfo= _CamProject._VisionInfo.Where(x => x.VisionName.Contains("Big")).FirstOrDefault();
            var regionData= _CamProject.ListRegions.Where(x => x.Color == "green").FirstOrDefault();
            if (regionData == null) return;
            var region = regionData.getRegion();
            var newImage= hImage.ReduceDomain(region);
            var ListWorkPiece = FindShapeModel(newImage, BigInfo);
            var RES= processed(hImage, ListWorkPiece, _CamProject._VisionInfo, _CamProject.ListRegions);
            RES.ElapsedTime = st.ElapsedMilliseconds.ToString();
            RES.ROIExistCount = ListWorkPiece.Count;
             EventFindRunResult?.Invoke(RES);
        }

    }
}
