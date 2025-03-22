
                                          /*            Thư viện Capacitive_Soil được viết bởi Trương Quang Bảo Khanh                     */

                        /*  Thư viện này được sử dụng với mục đích chuyển đổi tín hiệu analog thành giá trị % độ ẩm đất sử dụng cảm biến Độ ẩm đất điện dung */

                                          /*Các giá trị analog VD: 2751, 2325 v.v... là những giá trị được đo được từ % độ ẩm đất thực tế.*/

                                          /*Các bạn có thể đo các giá trị này bằng cách phơi khô 1 phần đất, sử dụng cân tiểu ly để ước lượng
                                            và suy ra giá trị phần trăm đất với công thức W = (mw x 100)/ms, trong đó mw là khối lượng nước
                                            và ms là khối lượng đất khô.                                                                   */
                                                                                                                        
#define SOIL 33

uint16_t convert_SOIL()
{
  uint16_t per_SOIL = 0;                            //biến này lưu trữ giá trị phần trăm độ ẩm đất
  uint16_t cambiendat = analogRead(SOIL);           //SOIL cần được khai báo là 1 pin cụ thể
  // độ ẩm đất bé hơn 0%
  if(cambiendat >= 2751)
  {
    per_SOIL = 0;
  }

  // độ ẩm đất đo được từ 0 - 20%
  else if(cambiendat >= 2325 && cambiendat <= 2750)
  {
    per_SOIL = map(cambiendat, 2325, 2750, 20, 0); //x = 0, y = 20; z = 2750, t = 2325; g = cambiendat(analog); %độ ẩm  = per_soils
  }

  // độ ẩm đất từ 20% - 40%
  else if(cambiendat >= 2020 && cambiendat <= 2324)
  {
    per_SOIL = map(cambiendat, 2020, 2324, 40, 20);
  }

  // độ ẩm đất từ 40% - 60%
  else if(cambiendat >= 1670 && cambiendat <= 2019)
  {
    per_SOIL = map(cambiendat, 1670, 2019, 60, 40);
  }

  // độ ẩm đất từ 60% - 80%
  else if(cambiendat >= 1520 && cambiendat <= 1669)
  {
    per_SOIL = map(cambiendat, 1520, 1669, 80, 60);
  }

  // độ ẩm đất từ 80% - 100%
  else if(cambiendat >= 1415 && cambiendat <= 1519)
  {
    per_SOIL = map(cambiendat, 1415, 1519, 100, 80);
  }

  // độ ẩm đất lớn hơn 100%
  else if(cambiendat <= 1414)
  {
    per_SOIL = 100;
  }
  return per_SOIL;
}