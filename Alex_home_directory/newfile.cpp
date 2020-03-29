#include <ncurses.h>
#include <unistd.h>  /* only for sleep() */

int kbhit(void)
{
    int ch = getch();

    if (ch != ERR) {
        ungetch(ch);
        return 1;
    } else {
        return 0;
    }
}

int main(void)
{
    int i = 0;
    int j = 0;
    int jcount = 0;
    int count = 0;
    initscr();
    nodelay(stdscr, TRUE);
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);

    scrollok(stdscr, TRUE);
    while (1) {
	if(j == 1) printw("button is triggered\n");
	else if(i%2 == 0) printw("end\n");

        if (kbhit()) {
            //printw("Key pressed! It was: %d\n", getch());
	    getch();
	    i = 0;
	    j = 1;
        jcount++;
        //printw("jcount is %d\n", jcount);
        refresh();
        } 
        

        else{
            count++;
            if (count <= 2) usleep(500000);
            else{
                count = 0;
	            i++;
	            j = 0;
                jcount = 0;
                refresh();
                usleep(300000);
            }
        }
    }
}
