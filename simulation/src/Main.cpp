#include "GL/freeglut.h"
#include <iostream>
#include "Constants.h"
#include "SPHSolver.h"

Visualization vis = Visualization::Default;

SPHSolver sph = SPHSolver();


void reshape(int width, int height) {
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, width / (float)height, 0.1, 100.0);
}

void drawParticle(float x, float y, float s)
{
	glBegin(GL_POLYGON);
	glVertex2f(x, y + s);
	glVertex2f(x - s, y);
	glVertex2f(x, y - s);
	glVertex2f(x + s, y);
	glEnd();
}

void draw_sph(SPHSolver& sph)
{
	for each (auto& particle in sph.particles)
	{
		glColor3f(0.5, 1.0, 1.0);
		drawParticle(particle.position[0], particle.position[1], 0.01);
		
	}
}

void display() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(-1.5f, -1.0f, -5.0f);

	draw_sph(sph);
	sph.update(Constants::TIMESTEP);

	glutSwapBuffers();
}
void idle() {
	display();
}
int main(int argc, char* argv[]) {
	
	glutInit(&argc, argv);
	glutInitWindowSize(800, 600);
	glutInitWindowPosition(50, 50);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	glEnable(GL_BLEND);
	glutCreateWindow("Physics Engine -- dtk");
	//Breakout.Init();
	glutDisplayFunc(&display);
	glutReshapeFunc(&reshape);
	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
	glutIdleFunc(&idle);
	glutMainLoop();
	return 0;
}