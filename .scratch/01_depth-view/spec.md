# DepthView

Status: ready-for-agent

## Problem Statement

RViz2 內建 Depth Cloud 的影像選擇流程需要先選 raw topic，再以 transport hint 指定傳輸方式，使用者難以直接選擇實際傳輸中的影像 topic。以 topic 名稱推測型態，也不適合經過重新命名的 topic。

使用者還需要降低深度影像轉換及顯示的運算負擔，以及排除超出關注深度的資料。目前希望在既有 arc_rviz_plugins 套件加入 DepthView，讓 topic 選擇、效能控制與深度濾除可直接設定。

## Solution

提供名為 DepthView 的 RViz2 display。使用者直接選擇深度影像、選用的 RGB 影像及獨立 CameraInfo topic；影像訊息型態依 ROS graph 公告判斷，壓縮編碼依訊息內容辨識，不需 transport hint。

DepthView 將深度轉為 Points 點雲，可設定每 N 張處理一次、像素抽樣步距及最大深度。可開關 RGB 著色；關閉、缺少或無法配對 RGB 時，繼續以固定深度色階顯示。畫面只保留最新一幀，避免歷史累積與處理佇列造成延遲。

## User Stories

1. As an RViz 使用者, I want a display named DepthView, so that I can 使用新的深度點雲顯示工具。
2. As an RViz 使用者, I want to 直接選擇實際發布的深度 topic, so that I can 不必另填 transport hint。
3. As an RViz 使用者, I want to 直接選擇實際發布的 RGB topic, so that I can 使用 raw 或壓縮影像。
4. As an RViz 使用者, I want topic 辨識依據 ROS graph 的訊息型態, so that I can 使用重新命名且沒有慣例後綴的 topic。
5. As an RViz 使用者, I want to 使用 Image 或 CompressedImage 深度輸入, so that I can 配合不同相機傳輸設定。
6. As an RViz 使用者, I want to 使用 Image 或 CompressedImage RGB 輸入, so that I can 選擇合適的彩色影像傳輸。
7. As an RViz 使用者, I want to 獨立選擇 CameraInfo, so that I can 明確指定與深度影像對應的相機內參。
8. As an RViz 使用者, I want to 在內參缺少或無效時看到原因, so that I can 修正設定而不觀看錯誤點雲。
9. As an RViz 使用者, I want to 使用毫米單位的 16UC1 深度, so that I can 正確顯示常見的整數深度影像。
10. As an RViz 使用者, I want to 使用公尺單位的 32FC1 深度, so that I can 正確顯示浮點深度影像。
11. As an RViz 使用者, I want 無效深度被略過, so that I can 避免零值、負值或非有限值形成錯誤點。
12. As an RViz 使用者, I want to 關閉 Color Image, so that I can 只使用深度並節省 RGB 接收與解碼成本。
13. As an RViz 使用者, I want to 使用已對齊 RGB 為點雲著色, so that I can 看出場景外觀。
14. As an RViz 使用者, I want 沒有 RGB 時仍能看到深度點雲, so that I can 使用只有深度輸出的相機。
15. As an RViz 使用者, I want RGB 尺寸不符時退回距離著色並提示, so that I can 繼續觀看深度並修正輸入。
16. As an RViz 使用者, I want to 配對時間最接近的 RGB, so that I can 降低彩色影像與深度的時間錯位。
17. As an RViz 使用者, I want to 調整 RGB 配對時間誤差, so that I can 配合來源的時間特性。
18. As an RViz 使用者, I want RGB 超出配對誤差時立即使用距離著色, so that I can 不因等待彩色影像而中斷更新。
19. As an RViz 使用者, I want to 每 N 張深度只處理一張, so that I can 降低解碼、點雲產生與資料更新成本。
20. As an RViz 使用者, I want to 設定最大深度, so that I can 隱藏例如超過 2 公尺的背景資料。
21. As an RViz 使用者, I want 最大深度設為 0 時不限制, so that I can 顯示所有有效深度。
22. As an RViz 使用者, I want to 以深度 Z 值濾除, so that I can 直接對照深度影像的數值。
23. As an RViz 使用者, I want to 設定 Pixel Stride, so that I can 用較少點數換取較低運算及繪製負擔。
24. As an RViz 使用者, I want to 使用 Points 並調整像素大小, so that I can 平衡可讀性與繪製成本。
25. As an RViz 使用者, I want to 設定固定深度色階範圍, so that I can 讓同一深度在場景變動時維持相同顏色。
26. As an RViz 使用者, I want 色階範圍與濾除範圍分開控制, so that I can 調整顏色而不意外隱藏點。
27. As an RViz 使用者, I want to 只顯示最新一幀, so that I can 避免歷史點雲累積。
28. As an RViz 使用者, I want 處理不及時只保留最新待處理深度, so that I can 避免顯示越來越延遲。
29. As an RViz 使用者, I want to 分別設定三種輸入的 Reliability 與 Durability, so that I can 配合相機、CameraInfo 及 rosbag 的 QoS。
30. As an RViz 使用者, I want 不支援或無法辨識的輸入有明確狀態提示, so that I can 排除設定問題而不依賴名稱猜測。

## Implementation Decisions

- 在既有 arc_rviz_plugins 套件新增 DepthView display，整合現有 Qt、pluginlib、RViz 與建置安裝方式。沿用套件的命名風格，對外顯示名稱為 DepthView。
- 深度與 RGB topic 選單依 ROS graph 篩選 Image、CompressedImage。訂閱採信實際公告的訊息型態；不從名稱或後綴推測，不要求 transport hint。
- CameraInfo topic 獨立選擇，依 CameraInfo 訊息型態篩選，不從影像 topic 推導名稱。必須提供與深度影像對應且有效的內參；尚未取得或無效時暫停產生點雲並提示原因。
- 參考既有 CameraInfo display 的 graph 型態辨識方式：型態未知時等待 discovery，型態不支援或不明確時顯示狀態，不猜測解碼器。既有 RGB 解碼實作不代表已具備 compressedDepth 支援。
- raw 深度支援 16UC1（毫米，轉成公尺）與 32FC1（公尺）。不支援的編碼明確報錯，不猜測單位。零值、負值、NaN、無限大均略過。
- 壓縮影像依 CompressedImage 的 format 與實際 payload 使用對應解碼器，涵蓋協議有效的 RGB 壓縮影像與 compressedDepth；不得將 compressedDepth 當作一般 JPEG/PNG 影像直接解碼。不支援、損壞或不相符的格式需明確報錯，不產生錯誤尺度的點雲。實作時依 Jazzy 官方協議確認支援的編碼組合。
- Use Color Image 可開關；關閉時取消 RGB 訂閱，停止 RGB 接收與解碼。沒有可用 RGB 時繼續顯示點雲，以深度著色。
- 第一版要求 RGB 與深度已對齊至相同像素座標，以相同像素取色。尺寸不同時退回深度著色並提示；尺寸相同不等於可自動證明幾何已對齊，對齊由輸入來源負責。
- RGB 以訊息時間戳選擇可用候選中最接近深度者；容許誤差可調，預設 0.1 秒。超出誤差或無 RGB 時不等待，直接以深度著色。配對候選快取必須有界，不能累積無限歷史。
- 降頻以收到的深度影像計數。N 為正整數，預設 1；N = 3 時略過前兩張，只將第三張交給解碼與點雲處理。跳過的深度不得先解碼再丟棄。此設定減少處理與更新頻率，不代表停止 RViz 對目前點雲的重繪。
- Max Depth (m) 使用相機光學座標的深度 Z，非到相機的歐氏距離；預設 0 表示不限制。大於限制的點剔除，等於限制的有效點保留。先判斷有效深度與限制，再產生 XYZ。
- Pixel Stride 為正整數，預設 1；水平與垂直皆以相同步距抽樣。步距 2 時點數約為四分之一，步距 3 時約為九分之一，實際數量受尺寸與有效深度影響。
- 使用 RViz 現有點雲渲染器的 Points 模式。Point Size (px) 可調，預設 3；實際支援大小依 GPU 與驅動能力。第一版不提供其他形狀。
- 深度著色使用固定範圍，Color Min Depth (m) 預設 0、Color Max Depth (m) 預設 5，近端藍、遠端紅；區間內平順變化，區間外使用端點顏色。這些設定不濾除點，獨立於 Max Depth。
- 只保留最新已完成的一幀點雲，新幀替換舊幀。深度處理若落後，待處理槽只保留最新通過降頻的深度，不建立逐幀累積佇列。此規則不禁止有界 RGB 配對快取。
- 深度、RGB、CameraInfo 的 QoS 分別提供 Reliability 與 Durability 設定，預設 Best Effort、Volatile。
- 與 RViz Fixed Frame／TF 機制整合，沿用標準 display 生命週期；渲染資源更新必須符合 RViz 執行緒要求。這是套件整合要求，不新增自訂座標系統。

## Testing Decisions

- 測試邊界已由使用者確認：以 DepthView 的完整輸入／輸出行為為主要邊界，輸入 ROS 影像、CameraInfo、時間戳與設定，驗證輸出點的位置、數量、顏色及狀態。避免測試私有方法、容器配置或固定呼叫順序。
- 優先從 display 的外部輸入與可觀察結果測試。若自動化必須隔離 GPU，僅引入一個位於完整影像處理入口／點雲輸出之間的高層邊界；不為每個處理步驟新增測試專用介面。
- 幾何案例使用已知內參與小型影像，確認投影、公尺尺度、16UC1／32FC1 一致性、無效值略過、最大 Z 邊界及像素抽樣。包括最大深度 0、不超過限制與剛超過限制的案例。
- 著色案例涵蓋 RGB 開關、已對齊像素取色、缺少 RGB、尺寸不符、時間容差內外與最近時間戳配對；深度色階需驗證端點、超界夾限及與濾除設定互不影響。
- 解碼案例涵蓋有效 raw RGB／深度、協議有效的壓縮 RGB／compressedDepth，以及不支援、損壞或 format 不相符的資料。驗證輸出尺度與顏色，不只驗證是否解碼成功。
- 以連續輸入驗證每 N 張處理第 N 張、略過資料不進入解碼處理，以及負載下不重播過時待處理幀。記憶體與延遲驗收涵蓋有界快取。
- ROS／RViz 整合驗證非慣例 topic 名稱、實際型態選擇、未知／不支援／多型態狀態、三種輸入 QoS、RGB 關閉後取消訂閱、CameraInfo 無效狀態及最新幀更新。
- RViz 畫面驗收確認 plugin 可載入、Points 顯示、3 px 預設與大小調整，以及 TF／Fixed Frame 下的空間位置。設定需可依一般 RViz 工作流程使用。
- 效能驗收在相同影像來源、解析度、硬體與場景下比較不同 N、Pixel Stride 與 RGB 開關，觀察處理時間、CPU／GPU 負擔、記憶體與顯示延遲。不先承諾固定加速倍數；不得以未測量的推論宣稱最省資源。
- Repo 現有先例為 CameraInfo display 的 topic discovery、影像轉換與狀態提示，以及既有 pluginlib／Qt 整合；目前未見自動化測試目標或測試檔。實作需加入必要的測試整合，不能聲稱已有測試框架可直接沿用。

## Out of Scope

- 未對齊 RGB／深度的跨相機投影、自動標定、遮擋處理與自動對齊。
- 自訂 GPU shader 或新的點雲渲染引擎。
- Squares、Spheres、Boxes 等其他繪製模式。
- 歷史點雲累積、地圖融合、錄製或發布 PointCloud2。
- 依 topic 名稱猜測型態或手動 transport hint 工作流程。
- 自動逐幀調整色階範圍、以歐氏距離取代 Z 濾除。
- 16UC1／32FC1 以外的 raw 深度編碼與無依據的單位推測。
- 未經量測的效能提升倍數保證。

## Further Notes

- 本規格來自 DepthView 設計討論與已確認的測試邊界，作為後續實作依據；本次只發布規格，不代表 plugin 已建立或通過驗收。
- 原始參考為 ROS 2 Jazzy 的內建 Depth Cloud：
  https://github.com/ros2/rviz/blob/jazzy/rviz_default_plugins/src/rviz_default_plugins/displays/depth_cloud/depth_cloud_display.cpp
- 使用者指定實作位置為 `arc_rviz_plugins/src`；本規格本身儲存於本機 Markdown issue tracker。
- 未明訂的設定細節（例如 Color Image 初始開關值、QoS history depth、RGB 快取容量、色階中間插值配色）屬實作時需記錄的選擇，不可當成已確認需求。設定需驗證有效範圍，例如 N 與 Pixel Stride 至少 1、時間誤差非負、色階上限大於下限。
- 開始實作前依當時官方 Jazzy 文件與協議核對影像解碼及 RViz API，尤其是 compressedDepth 的 format、payload 與不同深度編碼還原方式。
