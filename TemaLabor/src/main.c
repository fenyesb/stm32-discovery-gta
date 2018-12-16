/**
 ******************************************************************************
 * @file    main.c
 * @author  Ac6
 * @version V1.0
 * @date    01-December-2013
 * @brief   Default main function.
 ******************************************************************************
 */

#include <stm32f4xx.h>

#include "display.h"
#include "vcp.h"
#include "Common.h"
#include "drawing.h"
#include "touch.h"
#include "gyroscope.h"

#include "UartFifo.h"
#include "MessageHandler.h"
#include "CommHandler.h"

#include "WindowManager.h"
#include "MenuWindow.h"

#include <stdio.h>
#include <math.h>

static void SystemClock_Config(void);

#define max(a,b) ((a)>(b)?(a):(b))
#define min(a,b) ((a)<(b)?(a):(b))
#define limit(a,bottom,top) (max((bottom),min((top),(a))))
#define cutoff(a) ((a)>.5f||(a)<-.5f?a:0)

#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 320

int main(void) {

	SystemClock_Config();
	HAL_Init();
	DRV_InitLed(LED3);
	DRV_InitLed(LED4);
	DRV_Display_Init();
	DRV_Display_Clear();
	DRV_TS_Init();
	init_map();
	DRV_InitDrawTimer();

	while (1)
		;
}

volatile uint32_t elapsed_ticks = 0;

#define N_TIMES 5

#define MAP_WIDTH 70
#define MAP_HEIGHT 70

#define TILE_SIZE 32 //négyzet mérete

#define TILEMAP_LENGTH 9 //négyzettextúrák száma

extern const uint8_t map[];

extern unsigned long tilemap[]; //összes négyzettextúra

float phi; //kocsi iránya

float omega; //kocsi fordulási sebessége

float px, py; //kocsi pozíciója

float vx, vy; //kocsi sebessége

float cx, cy; //kamera pozíciója

double angle_diff(double a, double b) { //szögek különbsége -pi és pi között
	double diff = fmod(b - a + M_PI, 2 * M_PI);
	if (diff < 0)
		diff += 2 * M_PI;
	return diff - M_PI;

}

void physics() {

	int startx = max(0, (int)(px / TILE_SIZE) -2);
	int starty = max(0, (int)(py / TILE_SIZE) -2);
	int endx = min(MAP_WIDTH-1, (int)(px / TILE_SIZE) + 2);
	int endy = min(MAP_HEIGHT-1, (int)(py / TILE_SIZE) + 2);

	for(int x=startx;x <=endx; x++)
	{
		for(int y = starty; y<=endy;y++)
		{
			float centerx = x*32+16;
			float centery = y*32+16;
			float dx = px-centerx;
			float dy = py-centery;
			const float force = 1.5;
			float d = hypot(dx,dy);
			int id = map[y*MAP_WIDTH+x];
			if(d < 100 && (id == 0||id==1||id==2||id==3||id==7))
			{
				float ang = atan2(dy,dx);
				float f = max(0, 10-d/4) * force;
				vx += f * cos(ang);
				vy += f * sin(ang);
			}
		}
	}
}

void simulate(int touch, float an) { //kocsi mozgásának szimulációja

	physics();

	//érintés esetén gyorsítjuk az autót
	if (touch) {
		const float acceleration = 1;

		float delta_v_touch = cos(phi) * cos(an) + sin(phi) * sin(an);

		vx += cos(an) * delta_v_touch * acceleration;
		vy += sin(an) * delta_v_touch * acceleration;
	}

	//fékezés (gördülési ellenállás)
	const float resistance_linear = .05;
	vx -= vx * resistance_linear;
	vy -= vy * resistance_linear;

	//fékezés (légellenállás)
	const float resistance_quadratic = .01;
	vx -= (vx > 0 ? 1 : -1) * vx * vx * resistance_quadratic;
	vy -= (vy > 0 ? 1 : -1) * vy * vy * resistance_quadratic;

	//érintés esetén forgatjuk az autót
	if (touch) {
		const float rot_speed = .05;
		omega += angle_diff(phi, an) * rot_speed;
	}

	//forgatás lassítása
	const float rot_resistance = .2;
	omega -= omega * rot_resistance;

	//kanyarodás korlátozása
	const float rot_limit = .3;
	float rot_angle = limit(omega, -rot_limit, rot_limit);

	//sebességváltozás a kanyarodás irányába
	vx = vx * cos(rot_angle) + vy * -sin(rot_angle);
	vy = vx * sin(rot_angle) + vy * cos(rot_angle);

	//autó léptetése a sebessége alapján
	px += vx;
	py += vy;

	//autó forgatása a forgási sebesség alapján
	phi += omega;

	//tartomány korlátozása
	phi = fmod(phi + M_PI, 2 * M_PI);
	phi -= M_PI;
}

void init_map() { //pálya betöltése
	px = 390;
	py = 460;
}

extern const unsigned long car[]; //autó textúra

void draw_car_lq() //autó rajzolása (gyorsabb)
{
	float centerx = px - cx;
	float centery = py - cy;
	int startx = (int) centerx - 25;
	int starty = (int) centery - 25;
	int endx = (int) centerx + 24;
	int endy = (int) centery + 24;
	for (int x = startx; x <= endx; x += 2) {
		for (int y = starty; y <= endy; y += 2) {
			float dx = x - startx - 25; //delta
			float dy = y - starty - 25;
			float ang = -phi - M_PI / 2;
			float sx = dx * cos(ang) + dy * -sin(ang); //sampler
			float sy = dx * sin(ang) + dy * cos(ang);
			float tx = sx - 10;
			float ty = sy - 5;
			int tx_i = (int) (tx + 25);
			int ty_i = (int) (ty + 25);
			if (tx_i >= 0 && tx_i < 30 && ty_i >= 0 && ty_i < 40) {
				//BSP_LCD_DrawPixel(x, y, car[32 * ty_i + tx_i]);
				BSP_LCD_SetTextColor(car[32 * ty_i + tx_i]);
				BSP_LCD_FillRect(x, y, 2, 2);
			}
		}
	}
}

void draw_car() { //autó elforgatása és rajzolása
	float centerx = px - cx;
	float centery = py - cy;
	int startx = (int) centerx - 25;
	int starty = (int) centery - 25;
	int endx = (int) centerx + 24;
	int endy = (int) centery + 24;
	for (int x = startx; x <= endx; x++) {
		for (int y = starty; y <= endy; y++) {
			float dx = x - startx - 25; //delta
			float dy = y - starty - 25;
			float ang = -phi - M_PI / 2;
			float sx = dx * cos(ang) + dy * -sin(ang); //sampler
			float sy = dx * sin(ang) + dy * cos(ang);
			float tx = sx - 10;
			float ty = sy - 5;
			int tx_i = (int) (tx + 25);
			int ty_i = (int) (ty + 25);
			if (tx_i >= 0 && tx_i < 30 && ty_i >= 0 && ty_i < 40) {
				BSP_LCD_DrawPixel(x, y, car[32 * ty_i + tx_i]);
			}
		}
	}
}

void draw_tile(int ix, int offsetx, int offsety) { //egy négyzet rajzolása
	if (offsetx <= -TILE_SIZE || offsety <= -TILE_SIZE || offsetx >= 240
			|| offsety >= 320)
		return; //a négyzet a képernyőn kívül van
	else if (offsetx >= 0&& offsety >= 0 && offsetx < 240 - TILE_SIZE
	&& offsety < 320 - TILE_SIZE) {
		//a négyzet teljesen a képernyőn belül van
		BSP_LCD_DrawBitmap_Tile32(offsetx, offsety, ix, &tilemap);
	} else {
		//a négyzet egy része van a képernyőn belül
		for (int i = 0; i < TILE_SIZE; i++) {
			for (int j = 0; j < TILE_SIZE; j++) {
				if (offsetx + i >= 0 && offsetx + i < 240 && offsety + j >= 0
						&& offsety + j < 320) {
					BSP_LCD_DrawPixel(offsetx + i, offsety + j,
							tilemap[i + ix * TILE_SIZE * TILE_SIZE
									+ j * TILE_SIZE]);
				}
			}
		}
	}
}

void draw_map(int cx, int cy) { //térkép rajzolása
	int startx = max(0, cx / TILE_SIZE);
	int starty = max(0, cy / TILE_SIZE);
	int endx = min(MAP_WIDTH-1, (cx+SCREEN_WIDTH) / TILE_SIZE);
	int endy = min(MAP_WIDTH-1, (cy+SCREEN_HEIGHT) / TILE_SIZE);
	for (int x = startx; x <= endx; x++) {
		for (int y = starty; y <= endy; y++) {
			draw_tile(map[y * MAP_WIDTH + x], x * TILE_SIZE - cx,
					y * TILE_SIZE - cy);
		}
	}
}

uint16_t read_elapsed_ticks() { //eltelt idő a legutóbbi kiolvasás óta
	uint16_t current_time = TIM4->CNT;
	TIM4->CNT = 0;
	return current_time;
}

uint32_t time_frame_draw(uint16_t elapsed_ticks) { //rajzolási idő mérése és átlagolása
	static uint16_t times[N_TIMES];
	static uint16_t last_time = 0;
	static uint16_t ptr = 0;
	uint32_t sum = 0;
	for (uint16_t i = 0; i < N_TIMES; i++)
		sum += times[(i + ptr) % N_TIMES];
	times[ptr] = elapsed_ticks;
	ptr = (ptr + 1) % N_TIMES;
	return sum / N_TIMES;
}

void print_fps(uint16_t elapsed_ticks) { //átlagolt rajzolási idő kiírása
	uint32_t avg_frame_time = time_frame_draw(elapsed_ticks);
	BSP_LCD_SetTextColor(0xFF000000);
	BSP_LCD_SetBackColor(0xFFFFFFFF);
	static char buf[20];
	snprintf(buf, sizeof(buf), "%d ms (%d FPS)", (int) (avg_frame_time / 10),
			(int) (10000.0 / avg_frame_time));
	DRV_Display_WriteStringAt((Pixel ) { 10, 10 }, buf, ALIGN_Left);
}

void draw_frame() { //képernyő rajzolása
	uint16_t elapsed_ticks = read_elapsed_ticks();

	int touch = 0; //volt érintés?
	float an = 0; //milyen irányban volt az érintés?

	if (DRV_TS_IsTouchDetected()) {
		touch = 1;
		int x = DRV_TS_GetX();
		int y = SCREEN_HEIGHT - DRV_TS_GetY();
		an = -atan2(SCREEN_HEIGHT / 2 - y, -(SCREEN_WIDTH / 2 - x));
	}

	simulate(touch, an);

	//kamera szimuláció: mozgás irányába néz (camera_lookahead),
	//de követi az autót (camera_follow)
	const float camera_follow = .8;
	const float camera_lookahead = 3;
	cx -= cutoff(
			(cx - px - camera_lookahead*vx + SCREEN_WIDTH/2) * camera_follow);
	cy -= cutoff(
			(cy - py - camera_lookahead*vy + SCREEN_HEIGHT/2) * camera_follow);

	draw_map((int) cx, (int) cy);
	draw_car_lq();

	//diagnosztika
	/*BSP_LCD_SetTextColor(0xFFFF0000);
	int player_circle_x = (int) (px - cx);
	int player_circle_y = (int) (py - cy);
	BSP_LCD_DrawCircle(limit(player_circle_x, 10, 230),
			limit(player_circle_y, 10, 310), 5);
	BSP_LCD_DrawLine(player_circle_x, player_circle_y,
			player_circle_x + (int) (10 * cos(phi)),
			player_circle_y + (int) (10 * sin(phi)));

	print_fps(elapsed_ticks);
	static char buf[20];
	snprintf(buf, sizeof(buf), "pos: x=%3d y=%3d", (int) px, (int) py);
	BSP_LCD_DisplayStringAtLine(0, buf);
	snprintf(buf, sizeof(buf), "vel: x=%3d y=%3d", (int) vx, (int) vy);
	BSP_LCD_DisplayStringAtLine(1, buf);
	snprintf(buf, sizeof(buf), "cam: x=%3d y=%3d", (int) cx, (int) cy);
	BSP_LCD_DisplayStringAtLine(2, buf);
	snprintf(buf, sizeof(buf), "angle: %d", (int) (phi * 1000));
	BSP_LCD_DisplayStringAtLine(3, buf);
	snprintf(buf, sizeof(buf), "touch an: %d", (int) (an * 1000));
	BSP_LCD_DisplayStringAtLine(4, buf);
	snprintf(buf, sizeof(buf), "rot sp: %d", (int) (omega * 1000));
	BSP_LCD_DisplayStringAtLine(5, buf);*/
	DRV_Display_SwitchBuffer();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM3) {
		draw_frame();
	}
}

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 180000000
 *            HCLK(Hz)                       = 180000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 4
 *            APB2 Prescaler                 = 2
 *            HSE Frequency(Hz)              = 8000000
 *            PLL_M                          = 8
 *            PLL_N                          = 360
 *            PLL_P                          = 2
 *            PLL_Q                          = 7
 *            VDD(V)                         = 3.3
 *            Main regulator output voltage  = Scale1 mode
 *            Flash Latency(WS)              = 5
 * @param  None
 * @retval None
 */
static void SystemClock_Config(void) {
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	/* Enable Power Control clock */
	__HAL_RCC_PWR_CLK_ENABLE()
	;

	/* The voltage scaling allows optimizing the power consumption when the device is
	 clocked below the maximum system frequency, to update the voltage scaling value
	 regarding system frequency refer to product datasheet.  */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/* Enable HSE Oscillator and activate PLL with HSE as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 360;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	/* Activate the Over-Drive mode */
	HAL_PWREx_EnableOverDrive();

	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
	 clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line) {
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	DRV_ErrorLoop();

}
#endif
