# Livox ozel nokta bulutu mesaji (yalnizca derleme icin)
std_msgs/Header header
uint64 timebase         # ilk noktanin zamani
uint32 point_num        # toplam nokta sayisi
uint8 lidar_id          # lidar cihaz numarasi
uint8[3] rsvd           # rezerve
CustomPoint[] points
