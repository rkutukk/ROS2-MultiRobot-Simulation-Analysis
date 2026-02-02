**# ROS 2 Multi-Robot Simulation \& Performance Analysis**



Bu proje, \*\*ROS 2 Jazzy\*\* ve \*\*Gazebo\*\* simülasyon ortamında çoklu robot yönetimi, navigasyon senaryoları ve sistem performans analizi üzerine odaklanmaktadır. Bir üniversite Gezgin Robotlara Giriş ders projesi kapsamında geliştirilmiştir.



**## 🚀 Proje Kapsamı**

\- \*\*Çoklu Robot Kurulumu:\*\* 3 farklı robotun (Burger, Waffle, Waffle Pi) aynı Gazebo dünyasında (`.world`) koordinatlı olarak konumlandırılması.

\- \*\*Nav2 Entegrasyonu:\*\* `Action Client` yapısı kullanılarak robotlara otonom hedef gönderilmesi ve geri bildirim (feedback) mekanizmalarının test edilmesi.



**## 🛠️ Teknik Zorluklar ve Çözümler (Problem Solving)**



\### 1. CPU Darboğazı ve Performans Optimizasyonu

Simülasyon sırasında 3 robotun aynı anda çalışması Docker konteynırında \*\*%448 CPU\*\* yüküne neden olmuştur. 

\- \*\*Çözüm:\*\* Gazebo simülasyonu `headless` (GUI olmadan) çalıştırılarak grafik işlem yükü azaltılmış, işlemci gücü `Nav2` ve `TF` hesaplamalarına yönlendirilmiştir.



\### 2. TF (Transform) Zaman Senkronizasyonu

Yüksek CPU yükü nedeniyle koordinat verilerinde `TF\_OLD\_DATA` hataları gözlemlenmiştir. 

\- \*\*Çözüm:\*\* `use\_sim\_time: True` parametresi optimize edilerek sensör verilerinin zaman damgaları senkronize edilmiş ve robotun haritadaki yerelleştirmesi (localization) stabilize edilmiştir.



\## 📁 Dosya Yapısı

\- `/urdf`: Özelleştirilmiş robot tanımları.

\- `/worlds`: Modifiye edilmiş Gazebo simülasyon dünyaları.

\- `/scripts`: Nav2 hedef gönderici ve robot kontrol scriptleri.

\- `/docs`: Sistem hiyerarşisini gösteren TF Tree (frames.pdf) çıktısı.



\## 📊 Analiz

Proje kapsamında elde edilen TF Tree analizi, robotun `map` -> `odom` -> `base\_link` hiyerarşisinin doğruluğunu kanıtlamaktadır.

