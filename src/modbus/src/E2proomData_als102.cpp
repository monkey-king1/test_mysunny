#include "fileout/E2proomData.h"
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>  
#include <unistd.h>

void E2proomData::Init_als102_E2proomData()
{
    als102_pingjun_min=E2POOM_ALG102_LASERIMAGEPOS_PINGJUN_MIN;
    als102_pingjun_use=E2POOM_ALG102_LASERIMAGEPOS_PINGJUN_USE;
    als102_pingjun_max=E2POOM_ALG102_LASERIMAGEPOS_PINGJUN_MAX;
    als102_exposure_time_min=E2POOM_ALG102_LASERIMAGEPOS_EXPOSURE_TIME_MIN;
    als102_exposure_time_max=E2POOM_ALG102_LASERIMAGEPOS_EXPOSURE_TIME_MAX;
    als102_exposure_time_use=E2POOM_ALG102_LASERIMAGEPOS_EXPOSURE_TIME_USE;
    als102_b_yanmofuzhu_min=E2POOM_ALG102_LASERIMAGEPOS_B_YANMOFUZHU_MIN;
    als102_b_yanmofuzhu_use=E2POOM_ALG102_LASERIMAGEPOS_B_YANMOFUZHU_USE;
    als102_b_yanmofuzhu_max=E2POOM_ALG102_LASERIMAGEPOS_B_YANMOFUZHU_MAX;
    als102_b_gudingquyu_min=E2POOM_ALG102_LASERIMAGEPOS_B_GUDINGQUYU_MIN;
    als102_b_gudingquyu_use=E2POOM_ALG102_LASERIMAGEPOS_B_GUDINGQUYU_USE;
    als102_b_gudingquyu_max=E2POOM_ALG102_LASERIMAGEPOS_B_GUDINGQUYU_MAX;
    als102_widthliantongdis_min=E2POOM_ALG102_LASERIMAGEPOS_WIDTHLIANTONGDIS_MIN;
    als102_widthliantongdis_use=E2POOM_ALG102_LASERIMAGEPOS_WIDTHLIANTONGDIS_USE;
    als102_widthliantongdis_max=E2POOM_ALG102_LASERIMAGEPOS_WIDTHLIANTONGDIS_MAX;
    als102_highliantongdis_min=E2POOM_ALG102_LASERIMAGEPOS_HIGHLIANTONGDIS_MIN;
    als102_highliantongdis_use=E2POOM_ALG102_LASERIMAGEPOS_HIGHLIANTONGDIS_USE;
    als102_highliantongdis_max=E2POOM_ALG102_LASERIMAGEPOS_HIGHLIANTONGDIS_MAX;
    als102_gujiaerzhi_min=E2POOM_ALG102_LASERIMAGEPOS_GUJIAERZHI_MIN;
    als102_gujiaerzhi_use=E2POOM_ALG102_LASERIMAGEPOS_GUJIAERZHI_USE;
    als102_gujiaerzhi_max=E2POOM_ALG102_LASERIMAGEPOS_GUJIAERZHI_MAX;
    als102_jiguanghight_min=E2POOM_ALG102_LASERIMAGEPOS_JIGUANGHIGHT_MIN;
    als102_jiguanghight_use=E2POOM_ALG102_LASERIMAGEPOS_JIGUANGHIGHT_USE;
    als102_jiguanghight_max=E2POOM_ALG102_LASERIMAGEPOS_JIGUANGHIGHT_MAX;
    als102_jiguanglong_min=E2POOM_ALG102_LASERIMAGEPOS_JIGUANGLONG_MIN;
    als102_jiguanglong_use=E2POOM_ALG102_LASERIMAGEPOS_JIGUANGLONG_USE;
    als102_jiguanglong_max=E2POOM_ALG102_LASERIMAGEPOS_JIGUANGLONG_MAX;
    als102_jiguangkuandu_min=E2POOM_ALG102_LASERIMAGEPOS_JIGUANGKUANDU_MIN;
    als102_jiguangkuandu_use=E2POOM_ALG102_LASERIMAGEPOS_JIGUANGKUANDU_USE;
    als102_jiguangkuandu_max=E2POOM_ALG102_LASERIMAGEPOS_JIGUANGKUANDU_MAX;
    als102_Updif_min=E2POOM_ALG102_LASERIMAGEPOS_UPDIF_MIN;
    als102_Updif_use=E2POOM_ALG102_LASERIMAGEPOS_UPDIF_USE;
    als102_Updif_max=E2POOM_ALG102_LASERIMAGEPOS_UPDIF_MAX;
    als102_Updifmin_min=E2POOM_ALG102_LASERIMAGEPOS_UPDIFMIN_MIN;
    als102_Updifmin_use=E2POOM_ALG102_LASERIMAGEPOS_UPDIFMIN_USE;
    als102_Updifmin_max=E2POOM_ALG102_LASERIMAGEPOS_UPDIFMIN_MAX;
    als102_Uplong_min=E2POOM_ALG102_LASERIMAGEPOS_UPLONG_MIN;
    als102_Uplong_use=E2POOM_ALG102_LASERIMAGEPOS_UPLONG_USE;
    als102_Uplong_max=E2POOM_ALG102_LASERIMAGEPOS_UPLONG_MAX;
    als102_Downdif_min=E2POOM_ALG102_LASERIMAGEPOS_DOWNDIF_MIN;
    als102_Downdif_use=E2POOM_ALG102_LASERIMAGEPOS_DOWNDIF_USE;
    als102_Downdif_max=E2POOM_ALG102_LASERIMAGEPOS_DOWNDIF_MAX;
    als102_Downdifmin_min=E2POOM_ALG102_LASERIMAGEPOS_DOWNDIFMIN_MIN;
    als102_Downdifmin_use=E2POOM_ALG102_LASERIMAGEPOS_DOWNDIFMIN_USE;
    als102_Downdifmin_max=E2POOM_ALG102_LASERIMAGEPOS_DOWNDIFMIN_MAX;
    als102_Downdlong_min=E2POOM_ALG102_LASERIMAGEPOS_DOWNDLONG_MIN;
    als102_Downdlong_use=E2POOM_ALG102_LASERIMAGEPOS_DOWNDLONG_USE;
    als102_Downdlong_max=E2POOM_ALG102_LASERIMAGEPOS_DOWNDLONG_MAX;
    als102_St_Down_min=E2POOM_ALG102_LASERIMAGEPOS_ST_DOWN_MIN;
    als102_St_Down_use=E2POOM_ALG102_LASERIMAGEPOS_ST_DOWN_USE;
    als102_St_Down_max=E2POOM_ALG102_LASERIMAGEPOS_ST_DOWN_MAX;
    als102_Ed_Down_min=E2POOM_ALG102_LASERIMAGEPOS_ED_DOWN_MIN;
    als102_Ed_Down_use=E2POOM_ALG102_LASERIMAGEPOS_ED_DOWN_USE;
    als102_Ed_Down_max=E2POOM_ALG102_LASERIMAGEPOS_ED_DOWN_MAX;
    als102_St_Up_min=E2POOM_ALG102_LASERIMAGEPOS_ST_UP_MIN;
    als102_St_Up_use=E2POOM_ALG102_LASERIMAGEPOS_ST_UP_USE;
    als102_St_Up_max=E2POOM_ALG102_LASERIMAGEPOS_ST_UP_MAX;
    als102_Ed_Up_min=E2POOM_ALG102_LASERIMAGEPOS_ED_UP_MIN;
    als102_Ed_Up_use=E2POOM_ALG102_LASERIMAGEPOS_ED_UP_USE;
    als102_Ed_Up_max=E2POOM_ALG102_LASERIMAGEPOS_ED_UP_MAX;
    als102_Updif2_min=E2POOM_ALG102_LASERIMAGEPOS_UPDIF2_MIN;
    als102_Updif2_use=E2POOM_ALG102_LASERIMAGEPOS_UPDIF2_USE;
    als102_Updif2_max=E2POOM_ALG102_LASERIMAGEPOS_UPDIF2_MAX;
    als102_Updifmin2_min=E2POOM_ALG102_LASERIMAGEPOS_UPDIFMIN2_MIN;
    als102_Updifmin2_use=E2POOM_ALG102_LASERIMAGEPOS_UPDIFMIN2_USE;
    als102_Updifmin2_max=E2POOM_ALG102_LASERIMAGEPOS_UPDIFMIN2_MAX;
    als102_dis_center_st_min=E2POOM_ALG102_LASERIMAGEPOS_DIS_CENTER_ST_MIN;
    als102_dis_center_st_use=E2POOM_ALG102_LASERIMAGEPOS_DIS_CENTER_ST_USE;
    als102_dis_center_st_max=E2POOM_ALG102_LASERIMAGEPOS_DIS_CENTER_ST_MAX;
    als102_dis_center_ed_min=E2POOM_ALG102_LASERIMAGEPOS_DIS_CENTER_ED_MIN;
    als102_dis_center_ed_use=E2POOM_ALG102_LASERIMAGEPOS_DIS_CENTER_ED_USE;
    als102_dis_center_ed_max=E2POOM_ALG102_LASERIMAGEPOS_DIS_CENTER_ED_MAX;
    als102_b_opengudingdimian_min=E2POOM_ALG102_LASERIMAGEPOS_B_OPENGUDINGDIMIAN_MIN;          
    als102_b_opengudingdimian_max=E2POOM_ALG102_LASERIMAGEPOS_B_OPENGUDINGDIMIAN_MAX;         
    als102_b_opengudingdimian_use=E2POOM_ALG102_LASERIMAGEPOS_B_OPENGUDINGDIMIAN_USE;          
    als102_dimianpangdingjuli_min=E2POOM_ALG102_LASERIMAGEPOS_DIMIANPANGDINGJULI_MIN;          
    als102_dimianpangdingjuli_max=E2POOM_ALG102_LASERIMAGEPOS_DIMIANPANGDINGJULI_MAX;          
    als102_dimianpangdingjuli_use=E2POOM_ALG102_LASERIMAGEPOS_DIMIANPANGDINGJULI_USE;          
    als102_dimianpingjunshunum_min=E2POOM_ALG102_LASERIMAGEPOS_DIMIANPINGJUNSHUNUM_MIN;         
    als102_dimianpingjunshunum_max=E2POOM_ALG102_LASERIMAGEPOS_DIMIANPINGJUNSHUNUM_MAX;         
    als102_dimianpingjunshunum_use=E2POOM_ALG102_LASERIMAGEPOS_DIMIANPINGJUNSHUNUM_USE;    
    als102_dis_center_st2_min=E2POOM_ALG102_LASERIMAGEPOS_DIS_CENTER_ST2_MIN;   
    als102_dis_center_st2_max=E2POOM_ALG102_LASERIMAGEPOS_DIS_CENTER_ST2_MAX;  
    als102_dis_center_st2_use=E2POOM_ALG102_LASERIMAGEPOS_DIS_CENTER_ST2_USE;
    als102_dis_center_ed2_min=E2POOM_ALG102_LASERIMAGEPOS_DIS_CENTER_ED2_MIN;   
    als102_dis_center_ed2_max=E2POOM_ALG102_LASERIMAGEPOS_DIS_CENTER_ED2_MAX;  
    als102_dis_center_ed2_use=E2POOM_ALG102_LASERIMAGEPOS_DIS_CENTER_ED2_USE;
    als102_dis_center_st3_min=E2POOM_ALG102_LASERIMAGEPOS_DIS_CENTER_ST3_MIN;   
    als102_dis_center_st3_max=E2POOM_ALG102_LASERIMAGEPOS_DIS_CENTER_ST3_MAX;  
    als102_dis_center_st3_use=E2POOM_ALG102_LASERIMAGEPOS_DIS_CENTER_ST3_USE;
    als102_dis_center_ed3_min=E2POOM_ALG102_LASERIMAGEPOS_DIS_CENTER_ED3_MIN;   
    als102_dis_center_ed3_max=E2POOM_ALG102_LASERIMAGEPOS_DIS_CENTER_ED3_MAX;  
    als102_dis_center_ed3_use=E2POOM_ALG102_LASERIMAGEPOS_DIS_CENTER_ED3_USE;
    als102_xuexijuli_min=E2POOM_ALG102_LASERIMAGEPOS_XUEXIJULI_MIN;
    als102_xuexijuli_max=E2POOM_ALG102_LASERIMAGEPOS_XUEXIJULI_MAX;
    als102_xuexijuli_use=E2POOM_ALG102_LASERIMAGEPOS_XUEXIJULI_USE;
    als102_b_pingpowending_min=E2POOM_ALG102_LASERIMAGEPOS_B_PINGPOWENGDING_MIN;
    als102_b_pingpowending_max=E2POOM_ALG102_LASERIMAGEPOS_B_PINGPOWENGDING_MAX;
    als102_b_pingpowending_use=E2POOM_ALG102_LASERIMAGEPOS_B_PINGPOWENGDING_USE;
    als102_pingpowending_dis_min=E2POOM_ALG102_LASERIMAGEPOS_PINGPOWENGDING_DIS_MIN;
    als102_pingpowending_dis_max=E2POOM_ALG102_LASERIMAGEPOS_PINGPOWENGDING_DIS_MAX;
    als102_pingpowending_dis_use=E2POOM_ALG102_LASERIMAGEPOS_PINGPOWENGDING_DIS_USE;
    als102_b_xielvopen_min=E2POOM_ALG102_LASERIMAGEPOS_B_XIELVOPEN_MIN;
    als102_b_xielvopen_max=E2POOM_ALG102_LASERIMAGEPOS_B_XIELVOPEN_MAX;
    als102_b_xielvopen_use=E2POOM_ALG102_LASERIMAGEPOS_B_XIELVOPEN_USE;
    als102_xielvfanwei_min=E2POOM_ALG102_LASERIMAGEPOS_XIELVFANWEI_MIN;
    als102_xielvfanwei_max=E2POOM_ALG102_LASERIMAGEPOS_XIELVFANWEI_MAX;
    als102_xielvfanwei_use=E2POOM_ALG102_LASERIMAGEPOS_XIELVFANWEI_USE;
    als102_Uplong2_min=E2POOM_ALG102_LASERIMAGEPOS_UPLONG2_MIN;
    als102_Uplong2_use=E2POOM_ALG102_LASERIMAGEPOS_UPLONG2_USE;
    als102_Uplong2_max=E2POOM_ALG102_LASERIMAGEPOS_UPLONG2_MAX;
    als102_cebankongdongdis_min=E2POOM_ALG102_LASERIMAGEPOS_CEBANKONGDONGDIS_MIN;
    als102_cebankongdongdis_use=E2POOM_ALG102_LASERIMAGEPOS_CEBANKONGDONGDIS_USE;
    als102_cebankongdongdis_max=E2POOM_ALG102_LASERIMAGEPOS_CEBANKONGDONGDIS_MAX;
    als102_qiatouquweijuli_min=E2POOM_ALG102_LASERIMAGEPOS_QIATOUQUWEI_MIN;
    als102_qiatouquweijuli_use=E2POOM_ALG102_LASERIMAGEPOS_QIATOUQUWEI_USE;
    als102_qiatouquweijuli_max=E2POOM_ALG102_LASERIMAGEPOS_QIATOUQUWEI_MAX;
    als102_answerpoint_min=E2POOM_ALG102_LASERIMAGEPOS_ANSWERPOINT_MIN;
    als102_answerpoint_use=E2POOM_ALG102_LASERIMAGEPOS_ANSWERPOINT_USE;
    als102_answerpoint_max=E2POOM_ALG102_LASERIMAGEPOS_ANSWERPOINT_MAX;
}

void E2proomData::als102_check_para()
{
    if(als102_exposure_time<als102_exposure_time_min||als102_exposure_time>als102_exposure_time_max)
        als102_exposure_time=als102_exposure_time_use;
    if(als102_pingjun<als102_pingjun_min||als102_pingjun>als102_pingjun_max)
        als102_pingjun=als102_pingjun_use;
    if(als102_b_yanmofuzhu<als102_b_yanmofuzhu_min||als102_b_yanmofuzhu>als102_b_yanmofuzhu_max)
        als102_b_yanmofuzhu=als102_b_yanmofuzhu_use;
    if(als102_b_gudingquyu<als102_b_gudingquyu_min||als102_b_gudingquyu>als102_b_gudingquyu_max)
        als102_b_gudingquyu=als102_b_gudingquyu_use;
    if(als102_widthliantongdis<als102_widthliantongdis_min||als102_widthliantongdis>als102_widthliantongdis_max)
        als102_widthliantongdis=als102_widthliantongdis_use;
    if(als102_highliantongdis<als102_highliantongdis_min||als102_highliantongdis>als102_highliantongdis_max)
        als102_highliantongdis=als102_highliantongdis_use;
    if(als102_gujiaerzhi<als102_gujiaerzhi_min||als102_gujiaerzhi>als102_gujiaerzhi_max)
        als102_gujiaerzhi=als102_gujiaerzhi_use;
    if(als102_jiguanghight<als102_jiguanghight_min||als102_jiguanghight>als102_jiguanghight_max)
        als102_jiguanghight=als102_jiguanghight_use;
    if(als102_jiguanglong<als102_jiguanglong_min||als102_jiguanglong>als102_jiguanglong_max)
        als102_jiguanglong=als102_jiguanglong_use;
    if(als102_jiguangkuandu<als102_jiguangkuandu_min||als102_jiguangkuandu>als102_jiguangkuandu_max)
        als102_jiguangkuandu=als102_jiguangkuandu_use;
    if(als102_Updif<als102_Updif_min||als102_Updif>als102_Updif_max)
        als102_Updif=als102_Updif_use;
    if(als102_Updifmin<als102_Updifmin_min||als102_Updifmin>als102_Updifmin_max)
        als102_Updifmin=als102_Updifmin_use;
    if(als102_Uplong<als102_Uplong_min||als102_Uplong>als102_Uplong_max)
        als102_Uplong=als102_Uplong_use;
    if(als102_Downdif<als102_Downdif_min||als102_Downdif>als102_Downdif_max)
        als102_Downdif=als102_Downdif_use;
    if(als102_Downdifmin<als102_Downdifmin_min||als102_Downdifmin>als102_Downdifmin_max)
        als102_Downdifmin=als102_Downdifmin_use;
    if(als102_Downdlong<als102_Downdlong_min||als102_Downdlong>als102_Downdlong_max)
        als102_Downdlong=als102_Downdlong_use;    
    if(als102_St_Down<als102_St_Down_min||als102_St_Down>als102_St_Down_max)
        als102_St_Down=als102_St_Down_use;
    if(als102_Ed_Down<als102_Ed_Down_min||als102_Ed_Down>als102_Ed_Down_max)
        als102_Ed_Down=als102_Ed_Down_use;
    if(als102_St_Up<als102_St_Up_min||als102_St_Up>als102_St_Up_max)
        als102_St_Up=als102_St_Up_use;
    if(als102_Ed_Up<als102_Ed_Up_min||als102_Ed_Up>als102_Ed_Up_max)
        als102_Ed_Up=als102_Ed_Up_use;
    if(als102_Updif2<als102_Updif2_min||als102_Updif2>als102_Updif2_max)
        als102_Updif2=als102_Updif2_use;
    if(als102_Updifmin2<als102_Updifmin2_min||als102_Updifmin2>als102_Updifmin2_max)
        als102_Updifmin2=als102_Updifmin2_use;
    if(als102_dis_center_st<als102_dis_center_st_min||als102_dis_center_st>als102_dis_center_st_max)
        als102_dis_center_st=als102_dis_center_st_use;
    if(als102_dis_center_ed<als102_dis_center_ed_min||als102_dis_center_ed>als102_dis_center_ed_max)
        als102_dis_center_ed=als102_dis_center_ed_use;
    if(als102_b_opengudingdimian<als102_b_opengudingdimian_min||als102_b_opengudingdimian>als102_b_opengudingdimian_max)
        als102_b_opengudingdimian=als102_b_opengudingdimian_use;
    if(als102_dimianpangdingjuli<als102_dimianpangdingjuli_min||als102_dimianpangdingjuli>als102_dimianpangdingjuli_max)
        als102_dimianpangdingjuli=als102_dimianpangdingjuli_use;
    if(als102_dimianpingjunshunum<als102_dimianpingjunshunum_min||als102_dimianpingjunshunum>als102_dimianpingjunshunum_max)
        als102_dimianpingjunshunum=als102_dimianpingjunshunum_use;
    if(als102_dis_center_st2<als102_dis_center_st2_min||als102_dis_center_st2>als102_dis_center_st2_max)
        als102_dis_center_st2=als102_dis_center_st2_use;
    if(als102_dis_center_ed2<als102_dis_center_ed2_min||als102_dis_center_ed2>als102_dis_center_ed2_max)
        als102_dis_center_ed2=als102_dis_center_ed2_use;
    if(als102_dis_center_st3<als102_dis_center_st3_min||als102_dis_center_st3>als102_dis_center_st3_max)
        als102_dis_center_st3=als102_dis_center_st3_use;
    if(als102_dis_center_ed3<als102_dis_center_ed3_min||als102_dis_center_ed3>als102_dis_center_ed3_max)
        als102_dis_center_ed3=als102_dis_center_ed3_use;
    if(als102_xuexijuli<als102_xuexijuli_min||als102_xuexijuli>als102_xuexijuli_max)
        als102_xuexijuli=als102_xuexijuli_use;
    if(als102_b_pingpowending<als102_b_pingpowending_min||als102_b_pingpowending>als102_b_pingpowending_max)
        als102_b_pingpowending=als102_b_pingpowending_use;
    if(als102_pingpowending_dis<als102_pingpowending_dis_min||als102_pingpowending_dis>als102_pingpowending_dis_max)
        als102_pingpowending_dis=als102_pingpowending_dis_use;
    if(als102_b_xielvopen<als102_b_xielvopen_min||als102_b_xielvopen>als102_b_xielvopen_max)
        als102_b_xielvopen=als102_b_xielvopen_use;
    if(als102_xielvfanwei<als102_xielvfanwei_min||als102_xielvfanwei>als102_xielvfanwei_max)
        als102_xielvfanwei=als102_xielvfanwei_use;
    if(als102_Uplong2<als102_Uplong2_min||als102_Uplong2>als102_Uplong2_max)
        als102_Uplong2=als102_Uplong2_use;
    if(als102_cebankongdongdis<als102_cebankongdongdis_min||als102_cebankongdongdis>als102_cebankongdongdis_max)
        als102_cebankongdongdis=als102_cebankongdongdis_use;
    if(als102_qiatouquweijuli<als102_qiatouquweijuli_min||als102_qiatouquweijuli>als102_qiatouquweijuli_max)
        als102_qiatouquweijuli=als102_qiatouquweijuli_use;
    if(als102_answerpoint<als102_answerpoint_min||als102_answerpoint>als102_answerpoint_max)
        als102_answerpoint=als102_answerpoint_use;
}

void E2proomData::als102_read_para(char *filename)
{
    Uint8 *buff=NULL;
    CFileOut fo;

    buff=new Uint8[E2POOM_ALG102_LASERIMAGEPOS_SAVEBUFF];
    if(buff==NULL)
        return;
    if(0 > fo.ReadFile(filename,buff,E2POOM_ALG102_LASERIMAGEPOS_SAVEBUFF))
    {
        init_als102_para();
        if(buff!=NULL)
        {
          delete []buff;
          buff=NULL;
        }
    }
    else
    {
      Uint16 *ui16_p;
      Int16 *i16_p;

      ui16_p = (Uint16*)buff;
      als102_exposure_time=*ui16_p;
      ui16_p++;
      i16_p = (Int16*)ui16_p;
      als102_pingjun=*i16_p;
      i16_p++;
      als102_b_yanmofuzhu=*i16_p;
      i16_p++;
      als102_b_gudingquyu=*i16_p;
      i16_p++;
      als102_widthliantongdis=*i16_p;
      i16_p++;
      als102_highliantongdis=*i16_p;
      i16_p++;
      als102_gujiaerzhi=*i16_p;
      i16_p++;
      als102_jiguanghight=*i16_p;
      i16_p++;
      als102_jiguanglong=*i16_p;
      i16_p++;
      als102_jiguangkuandu=*i16_p;
      i16_p++;
      als102_Updif=*i16_p;
      i16_p++;
      als102_Updifmin=*i16_p;
      i16_p++;
      als102_Uplong=*i16_p;
      i16_p++;
      als102_Downdif=*i16_p;
      i16_p++;
      als102_Downdifmin=*i16_p;
      i16_p++;
      als102_Downdlong=*i16_p;
      i16_p++;
      als102_St_Down=*i16_p;
      i16_p++;
      als102_Ed_Down=*i16_p;
      i16_p++;
      als102_St_Up=*i16_p;
      i16_p++;
      als102_Ed_Up=*i16_p;
      i16_p++;
      als102_Updif2=*i16_p;
      i16_p++;
      als102_Updifmin2=*i16_p;
      i16_p++;
      als102_dis_center_st=*i16_p;
      i16_p++;
      als102_dis_center_ed=*i16_p;
      i16_p++;
      als102_b_opengudingdimian=*i16_p;
      i16_p++;
      als102_dimianpangdingjuli=*i16_p;
      i16_p++;
      als102_dimianpingjunshunum=*i16_p;
      i16_p++;
      als102_dis_center_st2=*i16_p;
      i16_p++;
      als102_dis_center_ed2=*i16_p;
      i16_p++;
      als102_dis_center_st3=*i16_p;
      i16_p++;
      als102_dis_center_ed3=*i16_p;
      i16_p++;
      als102_xuexijuli=*i16_p;
      i16_p++;
      als102_b_pingpowending=*i16_p;
      i16_p++;
      als102_pingpowending_dis=*i16_p;
      i16_p++;
      als102_b_xielvopen=*i16_p;
      i16_p++;
      als102_xielvfanwei=*i16_p;
      i16_p++;
      als102_Uplong2=*i16_p;
      i16_p++;
      als102_cebankongdongdis=*i16_p;
      i16_p++;
      als102_qiatouquweijuli=*i16_p;
      i16_p++;
      als102_answerpoint=*i16_p;
      i16_p++;
    }
    if(buff!=NULL)
    {
      delete []buff;
      buff=NULL;
    }

}

void E2proomData::write_als102_para(char *filename)
{
    Uint8 *buff=NULL;
    CFileOut fo;

    check_para();
    buff=new Uint8[E2POOM_ALG102_LASERIMAGEPOS_SAVEBUFF];
    if(buff==NULL)
      return;

    Uint16 *ui16_p;
    Int16 *i16_p;

    ui16_p = (Uint16*)buff;
    *ui16_p=als102_exposure_time;
    ui16_p++;
    i16_p = (Int16*)ui16_p;
    *i16_p=als102_pingjun;
    i16_p++;
    *i16_p=als102_b_yanmofuzhu;
    i16_p++;
    *i16_p=als102_b_gudingquyu;
    i16_p++;
    *i16_p=als102_widthliantongdis;
    i16_p++;
    *i16_p=als102_highliantongdis;
    i16_p++;
    *i16_p=als102_gujiaerzhi;
    i16_p++;
    *i16_p=als102_jiguanghight;
    i16_p++;
    *i16_p=als102_jiguanglong;
    i16_p++;
    *i16_p=als102_jiguangkuandu;
    i16_p++;
    *i16_p=als102_Updif;
    i16_p++;
    *i16_p=als102_Updifmin;
    i16_p++;
    *i16_p=als102_Uplong;
    i16_p++;
    *i16_p=als102_Downdif;
    i16_p++;
    *i16_p=als102_Downdifmin;
    i16_p++;
    *i16_p=als102_Downdlong;
    i16_p++;
    *i16_p=als102_St_Down;
    i16_p++;
    *i16_p=als102_Ed_Down;
    i16_p++;
    *i16_p=als102_St_Up;
    i16_p++;
    *i16_p=als102_Ed_Up;
    i16_p++;
    *i16_p=als102_Updif2;
    i16_p++;
    *i16_p=als102_Updifmin2;
    i16_p++;
    *i16_p=als102_dis_center_st;
    i16_p++;
    *i16_p=als102_dis_center_ed;
    i16_p++;
    *i16_p=als102_b_opengudingdimian;
    i16_p++;
    *i16_p=als102_dimianpangdingjuli;
    i16_p++;
    *i16_p=als102_dimianpingjunshunum;
    i16_p++;
    *i16_p=als102_dis_center_st2;
    i16_p++;
    *i16_p=als102_dis_center_ed2;
    i16_p++;
    *i16_p=als102_dis_center_st3;
    i16_p++;
    *i16_p=als102_dis_center_ed3;
    i16_p++;
    *i16_p=als102_xuexijuli;
    i16_p++;
    *i16_p=als102_b_pingpowending;
    i16_p++;
    *i16_p=als102_pingpowending_dis;
    i16_p++;
    *i16_p=als102_b_xielvopen;
    i16_p++;
    *i16_p=als102_xielvfanwei;
    i16_p++;
    *i16_p=als102_Uplong2;
    i16_p++;
    *i16_p=als102_cebankongdongdis;
    i16_p++;
    *i16_p=als102_qiatouquweijuli;
    i16_p++;
    *i16_p=als102_answerpoint;
    i16_p++;

    fo.WriteFile(filename,buff,E2POOM_ALG102_LASERIMAGEPOS_SAVEBUFF);

    if(buff!=NULL)
    {
      delete []buff;
      buff=NULL;
    }
}

void E2proomData::init_als102_para()
{
    als102_exposure_time=als102_exposure_time_use;
    als102_pingjun=als102_pingjun_use;
    als102_b_yanmofuzhu=als102_b_yanmofuzhu_use;
    als102_b_gudingquyu=als102_b_gudingquyu_use;
    als102_widthliantongdis=als102_widthliantongdis_use;
    als102_highliantongdis=als102_highliantongdis_use;
    als102_gujiaerzhi=als102_gujiaerzhi_use;
    als102_jiguanghight=als102_jiguanghight_use;
    als102_jiguanglong=als102_jiguanglong_use;
    als102_jiguangkuandu=als102_jiguangkuandu_use;
    als102_Updif=als102_Updif_use;
    als102_Updifmin=als102_Updifmin_use;
    als102_Uplong=als102_Uplong_use;
    als102_Downdif=als102_Downdif_use;
    als102_Downdifmin=als102_Downdifmin_use;
    als102_Downdlong=als102_Downdlong_use;
    als102_St_Down=als102_St_Down_use;
    als102_Ed_Down=als102_Ed_Down_use;
    als102_St_Up=als102_St_Up_use;
    als102_Ed_Up=als102_Ed_Up_use;
    als102_Updif2=als102_Updif2_use;
    als102_Updifmin2=als102_Updifmin2_use;
    als102_dis_center_st=als102_dis_center_st_use;
    als102_dis_center_ed=als102_dis_center_ed_use;
    als102_b_opengudingdimian=als102_b_opengudingdimian_use;
    als102_dimianpangdingjuli=als102_dimianpangdingjuli_use;
    als102_dimianpingjunshunum=als102_dimianpingjunshunum_use;
    als102_dis_center_st2=als102_dis_center_st2_use;
    als102_dis_center_ed2=als102_dis_center_ed2_use;
    als102_dis_center_st3=als102_dis_center_st3_use;
    als102_dis_center_ed3=als102_dis_center_ed3_use;
    als102_xuexijuli=als102_xuexijuli_use;
    als102_b_pingpowending=als102_b_pingpowending_use;
    als102_pingpowending_dis=als102_pingpowending_dis_use;
    als102_b_xielvopen=als102_b_xielvopen_use;
    als102_xielvfanwei=als102_xielvfanwei_use;
    als102_Uplong2=als102_Uplong2_use;
    als102_cebankongdongdis=als102_cebankongdongdis_use;
    als102_qiatouquweijuli=als102_qiatouquweijuli_use;
    als102_answerpoint=als102_answerpoint_use;
}