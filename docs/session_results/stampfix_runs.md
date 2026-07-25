## 追記9セッション: 最終検証run要約 (2026-07-22)

### 本番構成(診断オフ・負荷なし〜中、スタンプ修正+ZOH込み)
- final_r1 (slow=56):  位置誤差: mean=0.89mm RMSE=1.02mm max=5.15mm ヨー誤差: mean=0.038deg RMSE=0.172deg max=3.633deg 
- final_r2 (slow=82):  位置誤差: mean=0.90mm RMSE=1.04mm max=4.36mm ヨー誤差: mean=0.029deg RMSE=0.092deg max=2.780deg 
- final_r3 (slow=49):  位置誤差: mean=0.88mm RMSE=1.01mm max=3.87mm ヨー誤差: mean=0.026deg RMSE=0.118deg max=2.383deg 

### スタンプ修正の負荷耐性(意図的CPU負荷、diag有効)
- stampfix_load1 (slow=428):  位置誤差: mean=0.89mm RMSE=1.03mm max=4.26mm
- stampfix_load2 (slow=640):  位置誤差: mean=0.90mm RMSE=1.04mm max=6.01mm
- stampfix_load3 (slow=397):  位置誤差: mean=0.89mm RMSE=1.02mm max=4.97mm
