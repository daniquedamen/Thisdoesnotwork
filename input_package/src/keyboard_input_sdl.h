#include <SDL2/SDL.h>
#include <vector>

class KeyboardInput_SDL
{
	public:
		KeyboardInput_SDL();
		~KeyboardInput_SDL();
        std::vector<unsigned char> WhichKeysDown();
		bool get_quit();
    private:
        //Screen dimension constants
        const int SCREEN_WIDTH = 640;
        const int SCREEN_HEIGHT = 480;
        SDL_Window* gWindow;
		bool quit; // True if close icon of window is pressed.
};
