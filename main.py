import pygame
from pydualsense import pydualsense, TriggerModes
import numpy as np
import time
from typing import Tuple

# --- 定数 ---
# グリッド設定
GRID_WIDTH = 100
GRID_HEIGHT = 100

# ウィンドウ設定
WINDOW_SIZE = 400
CELL_WIDTH = WINDOW_SIZE // GRID_WIDTH
CELL_HEIGHT = WINDOW_SIZE // GRID_HEIGHT

# パーコレーション設定
# CRITICAL_POINT はmain関数内で変数として定義

# 色
COLOR_EMPTY = (0, 0, 0)
COLOR_OCCUPIED = (128, 128, 128)
COLOR_PERCOLATED = (0, 128, 255)
COLOR_TEXT = (200, 200, 200)

# セルの状態
EMPTY = 0
OCCUPIED = 1
PERCOLATED = 2

# --- クラス定義 ---

class PercolationSystem:
    """パーコレーションのシミュレーションロジックを管理するクラス"""
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
        self.grid = np.zeros((width, height), dtype=int)

    def update_grid(self, p: float):
        """占有率pに基づいてグリッドの状態を更新する"""
        random_grid = np.random.rand(self.width, self.height)
        self.grid = np.where(random_grid < p, OCCUPIED, EMPTY)

    def check_percolation(self) -> bool:
        """左端中央から右端への浸透が発生しているか判定する (BFSを使用)"""
        # PERCOLATED状態をリセット
        self.grid[self.grid == PERCOLATED] = OCCUPIED

        q = []
        # 左端の中央1マスのみを開始点とする
        center_y = self.height // 2
        self.grid[0, center_y] = PERCOLATED
        q.append((0, center_y))
        
        head = 0
        percolates = False
        while head < len(q):
            x, y = q[head]
            head += 1

            if x == self.width - 1:
                percolates = True
                # 右端に到達しても探索は続ける（浸透クラスター全体を色付けするため）

            # 隣接セルをチェック (上、下、左、右)
            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.width and 0 <= ny < self.height and self.grid[nx, ny] == OCCUPIED:
                    self.grid[nx, ny] = PERCOLATED
                    q.append((nx, ny))
        
        return percolates

    def draw(self, screen: pygame.Surface):
        """グリッドを画面に描画する"""
        for x in range(self.width):
            for y in range(self.height):
                color = COLOR_EMPTY
                if self.grid[x, y] == OCCUPIED:
                    color = COLOR_OCCUPIED
                elif self.grid[x, y] == PERCOLATED:
                    color = COLOR_PERCOLATED
                
                rect = pygame.Rect(x * CELL_WIDTH, y * CELL_HEIGHT, CELL_WIDTH, CELL_HEIGHT)
                pygame.draw.rect(screen, color, rect)

class ControllerManager:
    """DualSenseコントローラーの入出力を管理するクラス"""
    def __init__(self):
        try:
            self.ds = pydualsense()
            self.ds.init()
            print("DualSenseコントローラーが接続されました。")
        except Exception as e:
            print(f"エラー: DualSenseコントローラーが見つかりません。 {e}")
            self.ds = None
            raise ConnectionError("DualSense controller not found.")
        
        self.last_percolated_state = False
        self.percolation_flash_end_time = 0

    def close(self):
        """コントローラーを安全に閉じる"""
        if self.ds:
            self.ds.close()

    def get_r2_value(self) -> int:
        """R2トリガーの値を返す"""
        return self.ds.state.R2_value if self.ds else 0

    def get_lx_value(self) -> int:
        """左スティックのX軸の値を返す"""
        return self.ds.state.LX if self.ds else 0

    def get_rx_value(self) -> int:
        """右スティックのX軸の値を返す"""
        return self.ds.state.RX if self.ds else 0

    def get_dpad_down(self) -> bool:
        """DPADの下方向の値を返す"""
        return self.ds.state.DpadDown if self.ds else False
    
    def get_dpad_up(self) -> bool:
        """DPADの上方向の値を返す"""
        return self.ds.state.DpadUp if self.ds else False

    def get_cross_down(self) -> bool:
        """×ボタンの値を返す"""
        return self.ds.state.cross if self.ds else False

    def set_percolation_feedback(self, is_percolated: bool, p: float, p_range: Tuple[float, float], critical_point: float):
        """浸透の状態に応じてフィードバックを設定する"""
        if not self.ds:
            return

        # 1. ハプティックフィードバック (浸透中は常に振動)
        if is_percolated:
            self.ds.setRightMotor(200)
        else:
            self.ds.setRightMotor(0)

        # 2. アダプティブトリガー制御 (R2)
        if p < critical_point:
            force_factor = (p / critical_point) ** 4 if critical_point > 0 else 1.0
            force = int(255 * force_factor)
            self.ds.triggerR.setMode(TriggerModes.Rigid)
            self.ds.triggerR.setForce(1, force)
        else:
            self.ds.triggerR.setMode(TriggerModes.Rigid)
            self.ds.triggerR.setForce(1, 255)

        # 3. ライトバー制御
        if is_percolated:
            # 浸透中は白
            self.ds.light.setColorI(255, 255, 255)
        else:
            # pの値に応じて青から赤へ滑らかに変化
            lower_bound, upper_bound = p_range
            # pが範囲外にある場合を考慮して0-1にクリップ
            p_normalized = 0.0
            if (upper_bound - lower_bound) > 0:
                p_normalized = (p - lower_bound) / (upper_bound - lower_bound)
            p_normalized = max(0.0, min(1.0, p_normalized))

            r = int(255 * p_normalized)
            b = int(255 * (1 - p_normalized))
            self.ds.light.setColorI(r, 0, b)


def main():
    """メイン関数"""
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
    pygame.display.set_caption("パーコレーション臨界点体感アプリ")
    font = pygame.font.Font(None, 36)
    clock = pygame.time.Clock()

    try:
        controller = ControllerManager()
    except ConnectionError:
        # エラーメッセージを画面に表示
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
            screen.fill(COLOR_EMPTY)
            error_text = font.render("DualSense controller not found.", True, COLOR_TEXT)
            text_rect = error_text.get_rect(center=(WINDOW_SIZE // 2, WINDOW_SIZE // 2))
            screen.blit(error_text, text_rect)
            pygame.display.flip()
        pygame.quit()
        return

    perc_system = PercolationSystem(GRID_WIDTH, GRID_HEIGHT)
    
    running = True
    p = 0.0
    is_percolated = False

    # pの範囲と臨界点を動的に変更するための変数
    lower_bound = 0.0
    upper_bound = 1.0
    critical_point = 0.0
    selected_bound_idx = 0  # 0: lower, 1: upper, 2: critical
    bounds_text = ["Lower", "Upper", "Critical"]
    key_cooldown_time = 0
    KEY_COOLDOWN = 0.2 # seconds
    last_cross_state = False

    while running:
        now_time = time.time()
        # --- イベント処理 ---
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        # --- キーボード & D-Pad入力 ---
        if now_time > key_cooldown_time:
            keys = pygame.key.get_pressed()
            # D-Padや矢印キーで選択対象を切り替える
            if keys[pygame.K_UP] or controller.get_dpad_up():
                selected_bound_idx = (selected_bound_idx - 1 + 3) % 3
                key_cooldown_time = now_time + KEY_COOLDOWN
            elif keys[pygame.K_DOWN] or controller.get_dpad_down():
                selected_bound_idx = (selected_bound_idx + 1) % 3
                key_cooldown_time = now_time + KEY_COOLDOWN

        # --- コントローラー入力による状態更新 ---
        
        # ×ボタンで値を設定
        cross_is_pressed = controller.get_cross_down()
        if cross_is_pressed and not last_cross_state:
            if selected_bound_idx == 0: # lower bound
                lower_bound = p
                # lower_boundがupper_boundを超えた場合、upper_boundも引き上げる
                if lower_bound > upper_bound:
                    upper_bound = lower_bound
            elif selected_bound_idx == 1: # upper bound
                upper_bound = p
                # upper_boundがlower_boundを下回った場合、lower_boundも引き下げる
                if upper_bound < lower_bound:
                    lower_bound = upper_bound
            else: # critical point
                critical_point = p
        last_cross_state = cross_is_pressed
        
        # Bound値の調整 (右スティック)
        rx_value = controller.get_rx_value()
        ADJUST_SPEED = 0.001
        STICK_DEADZONE = 30 # ドリフト対策でデッドゾーンを拡大

        change = 0
        if rx_value < -STICK_DEADZONE: # 左に傾いている
            change = ADJUST_SPEED * (rx_value / 128.0) # rx_valueが負なのでchangeも負
        elif rx_value > STICK_DEADZONE: # 右に傾いている
            change = ADJUST_SPEED * (rx_value / 127.0)

        if change != 0:
            if selected_bound_idx == 0:  # lower bound
                lower_bound += change
                lower_bound = max(0.0, min(lower_bound, upper_bound))
            elif selected_bound_idx == 1:  # upper bound
                upper_bound += change
                upper_bound = max(lower_bound, min(1.0, upper_bound))
            else: # critical point
                critical_point += change
                critical_point = max(0.0, min(1.0, critical_point))
        
        # p値の計算 (R2トリガー)
        r2_value = controller.get_r2_value()
        p = lower_bound + (r2_value / 255.0) * (upper_bound - lower_bound)
        
        # 毎フレーム、グリッドを更新して浸透判定を行う
        perc_system.update_grid(p)
        is_percolated = perc_system.check_percolation()
        
        # --- フィードバックと描画 ---
        controller.set_percolation_feedback(is_percolated, p, (lower_bound, upper_bound), critical_point)
        
        # 画面描画
        screen.fill(COLOR_EMPTY)
        perc_system.draw(screen)
        
        # --- 情報表示 ---
        # テキスト背景の半透明パネル
        panel_height = 170
        info_panel = pygame.Surface((WINDOW_SIZE, panel_height), pygame.SRCALPHA)
        info_panel.fill((0, 0, 0, 180)) # 黒、70%の不透明度
        screen.blit(info_panel, (0, 0))

        # p値の表示
        p_text = font.render(f"p = {p:.4f}", True, COLOR_TEXT)
        screen.blit(p_text, (10, 10))

        # Bound & Critical表示
        bound_values = [lower_bound, upper_bound, critical_point]
        for i, bound_name in enumerate(bounds_text):
            is_selected = (i == selected_bound_idx)
            color = (255, 255, 0) if is_selected else COLOR_TEXT # 選択中は黄色
            bound_value = bound_values[i]
            prefix = "> " if is_selected else "  "
            text = font.render(f"{prefix}{bound_name}: {bound_value:.4f}", True, color)
            screen.blit(text, (10, 40 + i * 30))
        
        # 操作説明
        instruction_text1 = font.render("D-Pad/Arrow: Select", True, COLOR_TEXT)
        instruction_text2 = font.render("Right Stick: Adjust", True, COLOR_TEXT)
        instruction_text3 = font.render("Cross Button: Set to p", True, COLOR_TEXT)
        screen.blit(instruction_text1, (10, WINDOW_SIZE - 90))
        screen.blit(instruction_text2, (10, WINDOW_SIZE - 60))
        screen.blit(instruction_text3, (10, WINDOW_SIZE - 30))


        pygame.display.flip()
        clock.tick(60)

    controller.close()
    pygame.quit()

if __name__ == "__main__":
    main()
