#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
NAV2 AKSİYON İSTEMCİSİ VE HATA YÖNETİMİ SENARYOLARI
=====================================================

Bu script, ROS 2 Navigation Stack (Nav2) için bir aksiyon istemcisi (Action Client) oluşturur.
Aşağıda, bir robotun hedefe giderken karşılaşabileceği senaryolar ve bunların kod tarafındaki karşılıkları açıklanmıştır.

SENARYOLAR VE ÇÖZÜMLERİ:
------------------------

1. SENARYO: AKSİYON SUNUCUSU ÇALIŞMIYOR (Server Not Available)
   - Durum: Nav2 stack henüz başlatılmamış veya çökmüş olabilir.
   - Kod Karşılığı: `client.wait_for_server()` fonksiyonu.
   - Açıklama: İstemci, sunucu (server) aktif olana kadar bekler. Eğer belirli bir süre içinde sunucu gelmezse,
     program kullanıcıya bilgi verip güvenli bir şekilde sonlanabilir veya beklemeye devam edebilir.

2. SENARYO: HEDEF KABUL EDİLMEDİ (Goal Rejected)
   - Durum: Gönderilen hedef nokta harita sınırlarının dışında olabilir, geçerli bir navigasyon alanı olmayabilir
     veya sunucu o an başka bir işlemle meşgul olup yeni isteği reddedebilir.
   - Kod Karşılığı: `goal_handle.accepted` kontrolü.
   - Açıklama: Hedef gönderildikten sonra sunucudan "kabul" veya "ret" cevabı döner. Eğer `goal_handle.accepted`
     False ise, hedef reddedilmiş demektir. Bu durumda kullanıcıya "Hedef geçersiz" uyarısı verilir.

3. SENARYO: ROBOT YOLDA TIKANIKLIK YAŞADI VEYA PLAN OLUŞTURAMADI (Aborted)
   - Durum:
     a) Hedef geçerli fakat robot oraya giden bir yol bulamıyor (Global Planner hatası).
     b) Robot yola çıktı fakat dinamik bir engel (insan, eşya) yolunu kapattı ve kurtarma (recovery) davranışları da başarısız oldu.
     c) Robot sıkıştı (stuck).
   - Kod Karşılığı: `result.status == GoalStatus.STATUS_ABORTED`.
   - Açıklama: Aksiyon sonucu (result) döndüğünde durum kontrol edilir. Eğer durum ABORTED ise, robotun görevi
     başarısızlıkla sonuçlandırdığı anlaşılır.

4. SENARYO: HEDEF İPTAL EDİLDİ (Canceled)
   - Durum: Robot hareket halindeyken kullanıcı veya başka bir üst seviye yazılım "Dur" emri verdi.
   - Kod Karşılığı: `result.status == GoalStatus.STATUS_CANCELED`.
   - Açıklama: Görev yarıda kesilmiştir.

5. SENARYO: BAŞARI (Succeeded)
   - Durum: Robot hedefe başarıyla ulaştı.
   - Kod Karşılığı: `result.status == GoalStatus.STATUS_SUCCEEDED`.
   - Açıklama: Görev başarıyla tamamlandı.

-----------------------------------------------------
KOD UYGULAMASI AŞAĞIDADIR
-----------------------------------------------------
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

class RobustNav2Client(Node):
    def __init__(self):
        super().__init__('robust_nav2_client')
        
        # 1. Nav2 NavigateToPose aksiyonu için istemci oluşturuyoruz.
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, yaw):
        """
        Hedef noktaya gitme emri gönderir ve tüm olasılıkları yönetir.
        """
        
        # --- SENARYO 1: SUNUCU KONTROLÜ ---
        # Nav2 sunucusunun ayakta olup olmadığını kontrol ediyoruz.
        self.get_logger().info('Nav2 aksiyon sunucusu aranıyor...')
        server_reached = self._action_client.wait_for_server(timeout_sec=10.0)
        
        if not server_reached:
            self.get_logger().error('HATA: Nav2 aksiyon sunucusuna bağlanılamadı! Nav2 çalışıyor mu?')
            return # Sunucu yoksa işlem yapamayız, fonksiyondan çıkıyoruz.

        # Hedef mesajını oluşturuyoruz
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Pozisyon
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # Oryantasyon (Basitlik için Z ekseninde dönüş - Quaternion)
        # Not: Gerçek uygulamada Euler->Quaternion dönüşümü yapılmalıdır.
        # Burada örnek olarak 0 döndürme (sabit) veriyoruz.
        goal_msg.pose.pose.orientation.w = 1.0
        goal_msg.pose.pose.orientation.z = 0.0

        self.get_logger().info(f'Hedef gönderiliyor: X={x}, Y={y}...')

        # Hedefi asenkron olarak gönderiyoruz
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        # Hedefin sunucu tarafından KABUL edilip edilmediğini bekliyoruz
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Sunucunun hedefi kabul edip etmediğini kontrol eden callback.
        """
        goal_handle = future.result()

        # --- SENARYO 2: HEDEF KABUL/RET DURUMU ---
        if not goal_handle.accepted:
            # Hedef sunucu tarafından reddedildi (örn: harita dışı, geçersiz)
            self.get_logger().error('HATA: Hedef sunucu tarafından REDDEDİLDİ. (Geçersiz hedef veya sunucu meşgul)')
            return

        self.get_logger().info('BAŞARILI: Hedef sunucu tarafından KABUL EDİLDİ. Robot harekete başlıyor...')

        # Sonucu beklemek için bir "future" oluşturuyoruz
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Aksiyon tamamlandığında (başarılı, başarısız veya iptal) çağrılan callback.
        """
        result = future.result().result
        status = future.result().status

        # --- SENARYO 3, 4, 5: SONUÇ DEĞERLENDİRME ---
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            # SENARYO 5: BAŞARI
            self.get_logger().info('SONUÇ: HEDEFE ULAŞILDI! Görev Başarılı.')
            
        elif status == GoalStatus.STATUS_ABORTED:
            # SENARYO 3: ABORTED (Yol tıkalı, plan yapılamadı, robot sıkıştı)
            self.get_logger().error('SONUÇ: GÖREV İPTAL EDİLDİ (ABORTED).')
            self.get_logger().error('Sebepler: Yol bulunamadı, engel var veya robot sıkıştı.')
            
        elif status == GoalStatus.STATUS_CANCELED:
            # SENARYO 4: İPTAL
            self.get_logger().warn('SONUÇ: Görev kullanıcı tarafından İPTAL EDİLDİ.')
            
        else:
            # Diğer durumlar (Bilinmeyen hata vb.)
            self.get_logger().error(f'SONUÇ: Bilinmeyen durum kodu: {status}')

        # İşlem bittiği için node'u kapatabiliriz (demo amaçlı)
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        """
        Robot hareket halindeyken periyodik olarak çağrılan geri bildirim fonksiyonu.
        """
        feedback = feedback_msg.feedback
        # Kalan mesafeyi yazdıralım
        self.get_logger().info(f'Geri Bildirim: Hedefe kalan mesafe: {feedback.distance_remaining:.2f} metre')

def main(args=None):
    rclpy.init(args=args)

    action_client = RobustNav2Client()

    # Örnek bir hedef gönderelim (X=2.0, Y=0.5)
    # Bu değerleri kendi haritanıza göre değiştirebilirsiniz.
    action_client.send_goal(2.0, 0.5, 0.0)

    # Node'u ayakta tutuyoruz (callback'lerin çalışması için)
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        # rclpy.shutdown() get_result_callback içinde çağrıldığı için
        # burada hata almamak adına kontrol ediyoruz.
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
