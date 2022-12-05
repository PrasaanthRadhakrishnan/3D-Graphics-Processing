/*
 ===============================================================================
 Name        : DrawLine.c
 Author      : $Prasaanth Radhakrishnan
 Version     :
 Copyright   : $(copyright)
 Description : main definition
 ===============================================================================
 */

#include <cr_section_macros.h>
#include <stdbool.h>
#include <NXP/crp.h>
#include "LPC17xx.h"                        /* LPC17xx definitions */
#include "ssp.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* Be careful with the port number and location number, because

 some of the location may not exist in that port. */

#define PORT_NUM            0

uint8_t src_addr[SSP_BUFSIZE];
uint8_t dest_addr[SSP_BUFSIZE];

#define ST7735_TFTWIDTH 127
#define ST7735_TFTHEIGHT 159

#define ST7735_CASET 0x2A
#define ST7735_RASET 0x2B
#define ST7735_RAMWR 0x2C
#define ST7735_SLPOUT 0x11
#define ST7735_DISPON 0x29

#define swap(x, y) {x = x + y; y = x - y; x = x - y ;}

// defining color values

#define LIGHTBLUE 0x00FFE0
#define GOLD 0xFFD700
#define GREEN 0x00FF00
#define DARKBLUE 0x000033
#define BLACK 0x000000
#define BLUE 0x0007FF
#define RED 0xFF0000
#define MAGENTA 0x00F81F
#define WHITE 0xFFFFFF
#define PURPLE 0xCC33FF
#define BROWN 0x964B00
#define PI 3.1415926
#define SIZE 8

int _height = ST7735_TFTHEIGHT;
int _width = ST7735_TFTWIDTH;

bool trunkFlag = true;

void spiwrite(uint8_t c)

{

	int pnum = 0;

	src_addr[0] = c;

	SSP_SSELToggle(pnum, 0);

	SSPSend(pnum, (uint8_t*) src_addr, 1);

	SSP_SSELToggle(pnum, 1);

}

void writecommand(uint8_t c)

{

	LPC_GPIO0->FIOCLR |= (0x1 << 27);

	spiwrite(c);

}

void writedata(uint8_t c)

{

	LPC_GPIO0->FIOSET |= (0x1 << 27);

	spiwrite(c);

}

void writeword(uint16_t c)

{

	uint8_t d;

	d = c >> 8;

	writedata(d);

	d = c & 0xFF;

	writedata(d);

}

void write888(uint32_t color, uint32_t repeat)

{

	uint8_t red, green, blue;

	int i;

	red = (color >> 16);

	green = (color >> 8) & 0xFF;

	blue = color & 0xFF;

	for (i = 0; i < repeat; i++) {

		writedata(red);

		writedata(green);

		writedata(blue);

	}

}

void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)

{

	writecommand(ST7735_CASET);

	writeword(x0);

	writeword(x1);

	writecommand(ST7735_RASET);

	writeword(y0);

	writeword(y1);

}

void fillrect(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint32_t color)

{

	int16_t i;

	int16_t width, height;

	width = x1 - x0 + 1;

	height = y1 - y0 + 1;

	setAddrWindow(x0, y0, x1, y1);

	writecommand(ST7735_RAMWR);

	write888(color, width * height);

}

void lcddelay(int ms)

{

	int count = 24000;

	int i;

	for (i = count * ms; i--; i > 0)
		;

}

void lcd_init()

{

	int i;
	printf("LCD Demo Begins!!!\n");
	// Set pins P0.16, P0.27, P0.22 as output
	LPC_GPIO0->FIODIR |= (0x1 << 16);

	LPC_GPIO0->FIODIR |= (0x1 << 27);

	LPC_GPIO0->FIODIR |= (0x1 << 22);

	// Hardware Reset Sequence
	LPC_GPIO0->FIOSET |= (0x1 << 22);
	lcddelay(500);

	LPC_GPIO0->FIOCLR |= (0x1 << 22);
	lcddelay(500);

	LPC_GPIO0->FIOSET |= (0x1 << 22);
	lcddelay(500);

	// initialize buffers
	for (i = 0; i < SSP_BUFSIZE; i++) {

		src_addr[i] = 0;
		dest_addr[i] = 0;
	}

	// Take LCD display out of sleep mode
	writecommand(ST7735_SLPOUT);
	lcddelay(200);

	// Turn LCD display on
	writecommand(ST7735_DISPON);
	lcddelay(200);

}

void drawPixel(int16_t x, int16_t y, uint32_t color)

{

	if ((x < 0) || (x >= _width) || (y < 0) || (y >= _height))

		return;

	setAddrWindow(x, y, x + 1, y + 1);

	writecommand(ST7735_RAMWR);

	write888(color, 1);

}

/*****************************************************************************


 ** Descriptions:        Draw line function

 **

 ** parameters:           Starting point (x0,y0), Ending point(x1,y1) and color

 ** Returned value:        None

 **

 *****************************************************************************/

void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint32_t color)

{

	int16_t slope = abs(y1 - y0) > abs(x1 - x0);

	if (slope) {

		swap(x0, y0);

		swap(x1, y1);

	}

	if (x0 > x1) {

		swap(x0, x1);

		swap(y0, y1);

	}

	int16_t dx, dy;

	dx = x1 - x0;

	dy = abs(y1 - y0);

	int16_t err = dx / 2;

	int16_t ystep;

	if (y0 < y1) {

		ystep = 1;

	}

	else {

		ystep = -1;

	}

	for (; x0 <= x1; x0++) {

		if (slope) {

			drawPixel(y0, x0, color);

		}

		else {

			drawPixel(x0, y0, color);

		}

		err -= dy;

		if (err < 0) {

			y0 += ystep;

			err += dx;

		}

	}

}

/*****************************************************************************

 ** Description:        2D Assignment

 *****************************************************************************/

void drawTrees(int x, int y, float lambda, float treeLen, int angle, int diff, int branches) {

	double x1 = x + treeLen * cos(angle * PI / 180);
	double y1 = y + treeLen * sin(angle * PI / 180);
	if(trunkFlag){
		drawLine(x, y, x1, y1, BLACK);
		drawLine(x+1, y, x1+1, y1, BLACK);
		drawLine(x-1, y, x1-1, y1, BLACK);
		trunkFlag = false;
	}
	else{
		drawLine(x, y, x1, y1, GREEN);
	}
	if (branches > 1) {
		drawTrees(x1, y1, lambda, treeLen * lambda, angle + diff, diff, branches - 1);
		drawTrees(x1, y1, lambda, treeLen * lambda, angle - diff, diff, branches - 1);
	}
}

void ScreenSaver(int16_t x1, int16_t y1, int16_t x2, int16_t y2, float lambda, int16_t limit) {
	fillrect(0, 0, ST7735_TFTWIDTH, ST7735_TFTHEIGHT, BLACK);
	int x3, y3, x4, y4;
	int xn1, xn2, yn1, yn2, xn3, yn3, xn4, yn4;
	int j = 0;
	int wx, wy;
	int col;
	col = 0;

	/* For colors */
	int colour[8];
	colour[0] = 0x00FFE0; colour[1] = 0x00FF00; colour[2] = 0x000033; colour[3] = 0xCC33FF;
	colour[4] = 0x0007FF; colour[5] = 0xFF0000; colour[6] = 0x00F81F; colour[7] = 0xFFFFFF;

	wx = -(y2 - y1);
	wy = (x2 - x1);

	x3 = x1 + wx; x4 = x2 + wx;
	y3 = y1 + wy; y4 = y2 + wy;

	for (int i = 0; i < 10; i++) {
		drawLine(x1, y1, x2, y2, colour[col]);
		drawLine(x2, y2, x4, y4, colour[col]);
		drawLine(x3, y3, x1, y1, colour[col]);
		drawLine(x3, y3, x4, y4, colour[col]);

		xn1 = x1 + lambda * (x2 - x1); yn1 = y1 + lambda * (y2 - y1);
		xn2 = x2 + lambda * (x4 - x2); yn2 = y2 + lambda * (y4 - y2);
		xn3 = x4 + lambda * (x3 - x4); yn3 = y4 + lambda * (y3 - y4);
		xn4 = x3 + lambda * (x1 - x3); yn4 = y3 + lambda * (y1 - y3);

		x1 = xn1; x2 = xn2; x3 = xn4; x4 = xn3;
		y1 = yn1; y2 = yn2; y3 = yn4; y4 = yn3;

		if ((j < limit-1) && (i == 9)) {
			lcddelay(400);
			col = rand() % 7;
			x1 = rand() % 120; x2 = rand() % 120; y1 = rand() % 120; y2 = rand() % 120;
			wx = -(y2 - y1);
			wy = (x2 - x1);
			x3 = x1 + wx; x4 = x2 + wx;
			y3 = y1 + wy; y4 = y2 + wy;
			i = 0;
			j = j + 1;
		}
	}
}

/**************************************************
 *
 ** Description:        3D Assignment
 **
 **************************************************/

struct point_coordinate{
	int x;
	int y;
};

// Xe - Ye - Ze
int camera_x = 200;
int camera_y = 200;
int camera_z = 300;

// Parameters are xw - yw - zw
struct point_coordinate world_to_viewer_coord (int x_world, int y_world, int z_world)
{
	int scrn_x, scrn_y, Dist=100, x_diff=ST7735_TFTWIDTH/2, y_diff=ST7735_TFTHEIGHT/2;
	double x_p, y_p, z_p, theta, phi, rho;
	struct point_coordinate screen;

	rho = sqrt((pow(camera_x,2))+(pow(camera_y,2))+ (pow(camera_z,2)));
	theta = acos(camera_x/sqrt(pow(camera_x,2)+pow(camera_y,2)));
	phi = acos(camera_z/sqrt(pow(camera_x,2)+pow(camera_y,2)+pow(camera_z,2)));

	// world to viewer
	x_p = (y_world*cos(theta))- (x_world*sin(theta));
	y_p = (z_world*sin(phi))- (x_world*cos(theta)*cos(phi))- (y_world*cos(phi)*sin(theta));
	z_p = rho- (y_world*sin(phi)*cos(theta))- (x_world*sin(phi)*cos(theta))- (z_world*cos(phi));

	//perspective projection
	scrn_x = x_p*Dist/z_p;
	scrn_y = y_p*Dist/z_p;
	scrn_x = x_diff+scrn_x;
	scrn_y = y_diff-scrn_y;

	screen.x = scrn_x;
	screen.y = scrn_y;

	return screen;
}

void draw_coordinates (float move)
{
	fillrect(0, 0, ST7735_TFTWIDTH, ST7735_TFTHEIGHT, BLACK);
	struct point_coordinate lcd;
	int x[4],y[4];
	lcd = world_to_viewer_coord (0,-move,0);
		x[0]=lcd.x;
		y[0]=lcd.y;
	lcd = world_to_viewer_coord (180,-move,0);
		x[1]=lcd.x;
		y[1]=lcd.y;
	lcd = world_to_viewer_coord (0,180-move,0);
		x[2]=lcd.x;
		y[2]=lcd.y;
	lcd = world_to_viewer_coord (0,-move,180);
		x[3]=lcd.x;
		y[3]=lcd.y;

	drawLine(x[0],y[0],x[1],y[1],RED);		//x axis
	drawLine(x[0],y[0],x[2],y[2],GREEN);	//y axis
	drawLine(x[0],y[0],x[3],y[3],BLUE);		//z axis
}


void draw_cube(int startX, int startY, int startZ, int size)
{
	struct point_coordinate lcd;
	//int x1,y1,x2,y2,x3,y3,x4,y4,x5,y5,x6,y6,x7,y7,x8,y8;
	int x[8],y[8];
	camera_x = 200;
	camera_y = 200;
	camera_z = 200;

	lcd = world_to_viewer_coord (startX,startY,(size+startZ));
		x[0]=lcd.x;
		y[0]=lcd.y;
	lcd = world_to_viewer_coord ((size+startX),startY,(size+startZ));
		x[1]=lcd.x;
		y[1]=lcd.y;
	lcd = world_to_viewer_coord ((size+startX),(size+startY),(size+startZ));
		x[2]=lcd.x;
		y[2]=lcd.y;
	lcd = world_to_viewer_coord (startX,(size+startY),(size+startZ));
		x[3]=lcd.x;
		y[3]=lcd.y;
	lcd = world_to_viewer_coord ((size+startX),startY,startZ);
		x[4]=lcd.x;
		y[4]=lcd.y;
	lcd = world_to_viewer_coord ((size+startX),(size+startY), startZ);
		x[5]=lcd.x;
		y[5]=lcd.y;
	lcd = world_to_viewer_coord (startX,(size+startY),startZ);
		x[6]=lcd.x;
		y[6]=lcd.y;
	lcd = world_to_viewer_coord (startX,startY,startZ);
		x[7]=lcd.x;
		y[7]=lcd.y;
	drawLine(x[0], y[0], x[1], y[1],GOLD);
	drawLine(x[1], y[1], x[2], y[2],GOLD);
	drawLine(x[2], y[2], x[3], y[3],GOLD);
	drawLine(x[3], y[3], x[0], y[0],GOLD);
	drawLine(x[1], y[1], x[4], y[4],GOLD);
	drawLine(x[4], y[4], x[5], y[5],GOLD);
	drawLine(x[5], y[5], x[2], y[2],GOLD);
	drawLine(x[5], y[5], x[6], y[6],GOLD);
	drawLine(x[6], y[6], x[3], y[3],GOLD);
	drawLine(x[4], y[4], x[7], y[7],GOLD);
	drawLine(x[6], y[6], x[7], y[7],GOLD);
	drawLine(x[0], y[0], x[7], y[7],GOLD);
}

void draw_sphere(float move) {
	struct point_coordinate lcd;
	int x[40], y[40], z;
	int pointer = 1;
	int lcdx[10][40], lcdy[10][40];

	int radius, maxRadius = 100;
	int i, j, l, lambda = 10;

	for (l = 0; l < 10; l++) {
		radius = 100;
		radius = radius - (l * lambda);
		x[0] = radius;
		j = 1;

		//To get first quadrant
		for (i = 0; i < 10; i++) {
			y[i] = sqrt((pow(radius, 2)) - (pow(x[i], 2)));
			x[i + 1] = x[i] - (radius/10);
		}
		//for second quadrant
		for (i = 10; i < 20; i++) {
			x[i] = -x[i - j];
			y[i] = y[i - j];
			j = j + 2;
		}
		j = 1;
		//for third quadrant
		for (i = 20; i < 30; i++) {
			x[i] = x[i - j];
			y[i] = -y[i - j];
			j = j + 2;
		}
		j = 1;
		//for fourth quadrant
		for (i = 30; i < 40; i++) {
			x[i] = -x[i - j];
			y[i] = y[i - j];
			j = j + 2;
		}
		for (i = 0; i < 40; i++) {
			z = sqrt( (pow(maxRadius,2)) - (pow(x[i],2)) - (pow(y[i],2)) );
			lcd = world_to_viewer_coord(x[i], y[i]-move, z);
			lcdx[l][i] = lcd.x;
			lcdy[l][i] = lcd.y;
		}
		for (i = 0; i < 39; i++) {
			drawLine(lcdx[l][i], lcdy[l][i], lcdx[l][i + 1], lcdy[l][i + 1],WHITE);
		}
	}

	for(l=0;l<9;l++){
		for(i=0;i<39;i++){
			drawLine(lcdx[l][i], lcdy[l][i], lcdx[l+1][i], lcdy[l+1][i],WHITE);
		}
	}
}


/********** End of 3d assignment *********/

/**************************************************
 *
 ** Description:        Diffused Reflection Assignment
 **
 **************************************************/

typedef struct Point
{
	float x;
	float y;
}Point;

int light_x = 40;
int light_y = 60;
int light_z = 220;

double reflectivity[3] = {0.8, 0, 0};

int create_color(double r, double g, double b)
{
	return (((int)r & 0xFF)<<16)+(((int)g & 0xFF)<<8)+((int) b&0xFF);
}

int round(float x)
{
	int ans = x * 10;
	if(ans % 10 >= 5){
		return (int)x + 1;
	}
	else{
		return (int)x;
	}
}

void draw_HorizontalLine(int16_t x, int16_t y, int16_t width, uint32_t color)
{
	drawLine(x, y, x+width-1, y, color);
}

#define display_scaling (41300.5)
#define display_shifting (-200.5)

double calculate_intensity(int16_t xPs, int16_t yPs, int16_t zPs, int16_t xPi, int16_t yPi, int16_t zPi, int16_t i) {
	double   cosVal;
	double r = sqrt( pow((zPs - zPi), 2) + pow((yPs - yPi), 2) + pow((xPs - xPi), 2));
	double rcos = sqrt(pow((zPs - zPi), 2));
	cosVal = rcos / r;
	return (reflectivity[i] * 255 * cosVal) / pow(r, 2);
}

void fill_Triangle(int16_t x0, int16_t y0,int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint32_t color) {
	int16_t x, y, j, l;
	if (y0 > y1) {
		swap(y0, y1);
		swap(x0, x1);
	}
	if (y1 > y2) {
		swap(y2, y1);
		swap(x2, x1);
	}
	if (y0 > y1) {
		swap(y0, y1);
		swap(x0, x1);
	}
	if(y0 == y2) {
		x = y = x0;
		if(x1 < x) x = x1;
		else if(x1 > y) y = x1;

		if(x2 < x) x = x2;
		else if(x2 > y) y = x2;
		draw_HorizontalLine(x,y0, y-x+1, color);
		return;
	}

	int16_t dx01 = x1 - x0, dy01 = y1 - y0, dx02 = x2 - x0, dy02 = y2 - y0, dx12 = x2 - x1, dy12 = y2 - y1;
	int32_t sa = 0, sb = 0;

	if(y1 == y2) l = y1;
	else l = y1-1;

	for(j=y0; j<=l; j++) {
		x = x0 + sa / dy01; y = x0 + sb / dy02; sa += dx01;
		sb += dx02;
		if(x > y) swap(x,y);
		draw_HorizontalLine(x, j, y-x+1, color);
	}
	sa = dx12 * (j - y1); sb = dx02 * (j - y0);
	for(; j<=y2; j++) {
		x = x1 + sa / dy12; y = x0 + sb / dy02;
		sa += dx12;
		sb += dx02;
		if(x > y) swap(x,y); draw_HorizontalLine(x, j, y-x+1, color);
	}
}

//-------------DDA Algorithm for drawLine----------------------------
void drawLineDDA(int16_t x0, int16_t y0, int16_t x1, int16_t y1, double kR1, double kR2)
{
	int dx, dy, steps, i;
	float xInc, yInc, x = x0, y = y0;
	double n = 0;

	dx = x1 - x0;
	dy = y1 - y0;

	if(fabs(dx) > fabs(dy)){
		steps = fabs(dx);
	}
	else{
		steps = fabs(dy);
	}

	n = (kR2 - kR1) / steps;

	drawPixel(round(x), round(y), create_color(kR1, 0, 0));

	xInc = (float)dx / (float)steps;
	yInc = (float)dy / (float)steps;

	for(i = 0; i < steps; i++){
		x += xInc;
		y += yInc;
		drawPixel(round(x), round(y), create_color((kR1 + n), 0, 0));
	}
}


void diffused_reflection(int size)
{
	struct point_coordinate tmp, c1, c2, c3, c4;

	double c_kR[4];
	uint32_t c_color[4];
//-------------DDA Algorithm for boundary----------------------------
	c_kR[0] = calculate_intensity(light_x, light_y, light_z, 0, 0, size, 0);
	c_kR[1] = calculate_intensity(light_x, light_y, light_z, size, 0, size, 0);
	c_kR[2] = calculate_intensity(light_x, light_y, light_z, 0, size, size, 0);
	c_kR[3] = calculate_intensity(light_x, light_y, light_z, size, size, size, 0);

	for(int i = 0; i < 4; i++){
		c_kR[i] = c_kR[i] * display_scaling + display_shifting;
		if(c_kR[i] >= 255){
			c_kR[i] = 255;
		}
		else if(c_kR[i] <= 20){
			c_kR[i] = 20;
		}
	}

	c1 = world_to_viewer_coord(0, 0, size + 10);
	c2 = world_to_viewer_coord(size, 0, size + 10);
	c3 = world_to_viewer_coord(size, size, size + 10);
	c4 = world_to_viewer_coord(0, size, size + 10);

	drawLineDDA(c1.x, c1.y, c2.x, c2.y, c_kR[0], c_kR[1]);
	drawLineDDA(c3.x, c3.y, c2.x, c2.y, c_kR[1], c_kR[2]);
	drawLineDDA(c3.x, c3.y, c4.x, c4.y, c_kR[2], c_kR[3]);
	drawLineDDA(c1.x, c1.y, c4.x, c4.y, c_kR[3], c_kR[0]);
//-------------Linear for interior top----------------------------
	for (int i = 1; i < size; i++){
		for (int j = 1; j < size; j++){
			tmp = world_to_viewer_coord(i, j, size + 10);
			double kR = calculate_intensity(light_x, light_y, light_z, i, j, size, 0);
			kR = kR * display_scaling + display_shifting;
			if(kR >= 255){
				kR = 255;
			}
			else if(kR <= 20){
				kR = 20;
			}

			uint32_t color;
			color = create_color(kR, 0, 0);

			drawPixel(tmp.x, tmp.y, color);
		}
	}
//-------------Linear for interior left----------------------------
	for (int i = 0; i <= size; i++){
		for (int j = 10; j <= size + 10; j++){
			tmp = world_to_viewer_coord (size,i,j);
			double kR = calculate_intensity(light_x, light_y, light_z, size, i, j, 0);
			kR = kR * display_scaling + display_shifting;
			if(kR >= 255){
				kR = 255;
			}
			else if(kR <= 20){
				kR = 20;
			}

			uint32_t color;
			color = create_color(kR, 0, 0);

			drawPixel(tmp.x, tmp.y, color);
		}
	}
//-------------Linear for interior right----------------------------
	for (int i = 0; i <= size; i++)
	{
		for (int j = 10; j <= size + 10; j++) {
			tmp = world_to_viewer_coord (i,size,j);
			double kR = calculate_intensity(light_x, light_y, light_z, i, size, j, 0);
			kR = kR * display_scaling + display_shifting;
			if(kR >= 255){
				kR = 255;
			}
			else if(kR <= 20){
				kR = 20;
			}
			uint32_t color;
			color = create_color(kR, 0, 0);

			drawPixel(tmp.x, tmp.y, color);
		}
	}
}

void shadow(double x[], double y[], double z[])
{
	int light_x = -20;
	int light_y = -20;
	int light_z = 220;
	camera_x = 200;
	camera_y = 200;
	camera_z = 300;

	int x_its[4]={0}, y_its[4]={0}, z_its[4]={0};
	struct point_coordinate s1,s2,s3,s4,l1;
	float lambda = (-1 * light_z) / (light_z - z[6]);
//-------------[6] = (size, size, size)----------------------------
	x_its[0] = light_x + lambda * (light_x - x[6]);
	y_its[0] = light_y + lambda * (light_y - y[6]);
	z_its[0] = light_z + lambda * (light_z - z[6]);
//-------------[5] = (size, 0, size)----------------------------
	x_its[1] = light_x + lambda * (light_x - x[5]);
	y_its[1] = light_y + lambda * (light_y - y[5]);
	z_its[1] = light_z + lambda * (light_z - z[5]);
//-------------[7] = (0, size, size)----------------------------
	x_its[2] = light_x + lambda * (light_x - x[7]);
	y_its[2] = light_y + lambda * (light_y - y[7]);
	z_its[2] = light_z + lambda * (light_z - z[7]);
//-------------[4] = (0, 0, size)----------------------------
	x_its[3] = light_x + lambda * (light_x - x[4]);
	y_its[3] = light_y + lambda * (light_y - y[4]);
	z_its[3] = light_z + lambda * (light_z - z[4]);

	s1 = world_to_viewer_coord(x_its[0],y_its[0],z_its[0]);
	s2 = world_to_viewer_coord(x_its[1],y_its[1],z_its[1]);
	s3 = world_to_viewer_coord(x_its[2],y_its[2],z_its[2]);
	s4 = world_to_viewer_coord(x_its[3],y_its[3],z_its[3]);

	l1 = world_to_viewer_coord(light_x,light_y,light_z);

//----------------shadow-------------------------
	drawLine(s1.x, s1.y, s2.x, s2.y, DARKBLUE);
	drawLine(s2.x, s2.y, s4.x, s4.y, DARKBLUE);
	drawLine(s3.x, s3.y, s4.x, s4.y, DARKBLUE);
	drawLine(s3.x, s3.y, s1.x, s1.y, DARKBLUE);
//---------------fill shadow-------------------------
	fill_Triangle(s1.x, s1.y, s2.x, s2.y, s4.x, s4.y,DARKBLUE);
	fill_Triangle(s1.x, s1.y, s3.x, s3.y, s4.x, s4.y,DARKBLUE);
//--------------light to shadow---------------------
	drawLine(l1.x, l1.y, s1.x, s1.y, GOLD);
	drawLine(l1.x, l1.y, s2.x, s2.y, GOLD);
	drawLine(l1.x, l1.y, s3.x, s3.y, GOLD);
	drawLine(l1.x, l1.y, s4.x, s4.y, GOLD);
}


//-------------3D tree----------------------------
Point rotate_point(Point start, Point end, float angle)
{
	Point post, rotate, pre;
	angle = angle * (PI/180);
	float sina = sinf(angle);
	float cosa = cosf(angle);
	//post
	post.x = start.x - end.x;
	post.y = start.y - end.y;
	//rotate
	rotate.x = post.x * cosa + post.y * -sina;
	rotate.y = post.x * sina + post.y * cosa;
	//pre
	pre.x = rotate.x + end.x;
	pre.y = rotate.y + end.y;
	return pre;
}

void tree_branch_3D(Point start, Point end, int y, float lamda, float angle, int level)
{
	struct point_coordinate d_m, d_l, d_r, d_e;
	Point mid, left, right;

	if(level == 0){
		return;
	}
	d_e = world_to_viewer_coord(end.x, y, end.y);
	//mid branch
	mid.x = end.x + lamda * (end.x - start.x);
	mid.y = end.y + lamda * (end.y - start.y);
	d_m = world_to_viewer_coord(end.x + lamda * (end.x - start.x), y, end.y + lamda * (end.y - start.y));

	drawLine(d_m.x, d_m.y, d_e.x, d_e.y, GREEN);

	//right branch
	right = rotate_point(mid, end, angle);
	d_r = world_to_viewer_coord(right.x, y, right.y);
	drawLine(d_r.x, d_r.y, d_e.x, d_e.y, GREEN);

	//left branch
	left = rotate_point(mid, end, 360 - angle);
	d_l = world_to_viewer_coord(left.x, y, left.y);
	drawLine(d_l.x, d_l.y, d_e.x, d_e.y, GREEN);

	tree_branch_3D(end, mid, y,lamda, angle, level - 1);
	tree_branch_3D(end, right, y, lamda, angle, level - 1);
	tree_branch_3D(end, left, y, lamda, angle, level - 1);
}

void tree_stem_3D(int start_px, int start_py, int start_pz, int end_px, int end_py, int end_pz, int color)
{
	struct point_coordinate ps, pe;
	ps = world_to_viewer_coord(start_px, start_py, start_pz);
	pe = world_to_viewer_coord(end_px, end_py, end_pz);
	drawLine(ps.x, ps.y, pe.x, pe.y, color);
}

//-------------S on top of sphere----------------------------
void draw_S(int start_x, int start_y, int start_z, int size, uint32_t color)
{
	int xs[6], ys[6], z = start_z + size;
	int xss[6], yss[6];
	int x, y;
	struct point_coordinate p0, p1, p2, p3, p4, p5;

	xs[0] = start_x + 10;
	ys[0] = start_y + 10;

	xs[1] = start_x + size - size / 3;
	ys[1] = ys[0];

	xs[2] = xs[1];
	ys[2] = ys[1] + size/3;

	xs[3] = xs[0];
	ys[3] = ys[2];

	xs[4] = xs[0];
	ys[4] = ys[3] + size/3;

	xs[5] = xs[1];
	ys[5] = ys[4];

	for(int i = -4; i < 5; i++){
		p1 = world_to_viewer_coord(xs[1] + i, ys[1], z);
		p0 = world_to_viewer_coord(xs[0] - i, ys[0], z);
		drawLine(p1.x, p1.y, p0.x, p0.y, color);
	}

	for(int i = -4; i < 5; i++){
		p1 = world_to_viewer_coord(xs[1] + i, ys[1], z);
		p2 = world_to_viewer_coord(xs[2], ys[2] + i, z);
		drawLine(p1.x, p1.y, p2.x, p2.y, color);
	}

	for(int i = -4; i < 5; i++){
		p5 = world_to_viewer_coord(xs[5] + i, ys[5], z);
		p4 = world_to_viewer_coord(xs[4], ys[4] + i, z);
		drawLine(p5.x, p5.y, p4.x, p4.y, color);
	}

	for(int i = -4; i < 5; i++){
		p3 = world_to_viewer_coord(xs[3] + i, ys[3], z);
		p4 = world_to_viewer_coord(xs[4], ys[4] + i, z);
		drawLine(p3.x, p3.y, p4.x, p4.y, color);
	}

	for(int i = -4; i < 5; i++){
		p2 = world_to_viewer_coord(xs[2] + i, ys[2], z);
		p3 = world_to_viewer_coord(xs[3], ys[3] + i, z);
		drawLine(p2.x, p2.y, p3.x, p3.y, color);
	}
}

/********** End of last assignment *********/

void operations() {
	int x0, y0;
	int num;
	int input = 1;
	float treeLen;

	/* Extras for assignment 3 */

	int start_x = 0;
	int start_y = 0;
	int start_z = 10;
	int size = 100;

	double x[8] = {start_x,(start_x+size),(start_x+size), start_x, start_x, (start_x+size),(start_x+size), start_x};
	double y[8] = {start_y, start_y, (start_y+size),(start_y+size), start_y, start_y, (start_y+size),(start_y+size)};
	double z[8] = {start_z, start_z, start_z, start_z, (start_z+size),(start_z+size),(start_z+size),(start_z+size)};

	while (input != 0) {

		printf("My name is Prasaanth Radhakrishnan, SJSU ID: 015279524\n");
		printf("CMPE 240 assignment\n");
		printf("Choose the operation to perform:\n");
		printf("1. Screen Saver \n2. Draw Forest \n3. Draw 3d coordinates and cube \n4. Draw Sphere \n");
		printf("5. Draw Sphere and cube \n6. Diffused reflection \n0. To Exit \n");
		scanf("%d", &input);
		printf("\n");

		switch (input) {

		case 0:
			break;

		case 1:
			printf("Enter the number of squares to print: \n");
			scanf("%d", &num);
			ScreenSaver(120, 30, 120, 140, 0.8, num);
			break;

		case 2:
			fillrect(0, 0, ST7735_TFTWIDTH, ST7735_TFTHEIGHT / 3, BROWN);
			fillrect(0, ST7735_TFTHEIGHT / 3, ST7735_TFTWIDTH, ST7735_TFTHEIGHT,
					DARKBLUE);
			printf("Enter the number of trees to print: \n");
			scanf("%d", &num);
			for (int i = 0; i < num; i++) {
				lcddelay(200);
				x0 = rand() % 150;
				y0 = rand() % 50;
				treeLen = rand() % 15;
				drawTrees(x0, y0, 0.8, treeLen, 90, 10, 10);
				trunkFlag = true;
			}
			break;

		case 3:
			draw_coordinates(0);
			draw_cube(100,100,100,50);
			break;

		case 4:
			draw_coordinates(0);
			draw_sphere(0);
			break;

		case 5:
			draw_coordinates(0);
			draw_cube(100,100,100,50);
			draw_sphere(0);
			break;

		/* Last Assignment */

		case 6:
			draw_coordinates(300);
			draw_sphere(300);
			draw_S(0 ,-300 ,0, size, DARKBLUE);
			//draw_cube(start_x,start_y,start_z,size);
			shadow(x, y, z);
			diffused_reflection(size);
			//draw_sphere();

			//tree_3D
			Point t_s, t_e;
			int length = 20;
			t_s.x = size / 2;
			t_s.y = start_z;
			t_e.x = size / 2;
			t_e.y = start_z + length;

			tree_stem_3D(size / 2, size, start_z, size / 2, size, start_z + length, BLACK);
			tree_branch_3D(t_s, t_e, size, 0.8, 30, 5);
			break;

		default:
			printf("Invalid Input\n");
		}
	}

	printf("Operations Completed\n");
}

/*

 Main Function main()

 */
int main(void)

{

	uint32_t pnum = PORT_NUM;

	pnum = 0;

	if (pnum == 0)
		SSP0Init();

	else
		puts("Port number is not correct");

	lcd_init();

	/* Function call to all operations */
	operations();
	return 0;
}

