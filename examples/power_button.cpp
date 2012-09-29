#include <libpowerbutton.h>

int main (int argc, char *argv[])
{
    printf("Button pressed %i times.\n",power_button_get_value());
    while(power_button_get_value()<5)
    {
        sleep(1);
        printf("Button pressed %i times.\n",power_button_get_value());
    }
    power_button_reset();
    return 0;
};
