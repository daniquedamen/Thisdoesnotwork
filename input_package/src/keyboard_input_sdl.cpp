#include <stdio.h>
#include <string>
#include <iostream>

#include "../include/keyboard_input/keyboard_input_sdl.h"


KeyboardInput_SDL::KeyboardInput_SDL():
	quit(false)
{
	//Initialize SDL
	if( SDL_Init( SDL_INIT_VIDEO ) < 0 ) // We need video because we need a window to send the key presses to.
	{
		printf( "SDL could not initialize! SDL Error: %s\n", SDL_GetError() );
        return;
	}

    //Create window
    gWindow = SDL_CreateWindow( "SDL Tutorial", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN );
    if( gWindow == NULL )
    {
        printf( "Window could not be created! SDL Error: %s\n", SDL_GetError() );
        return;
    }
}


KeyboardInput_SDL::~KeyboardInput_SDL()
{
	//Destroy window	
	SDL_DestroyWindow( gWindow );
	gWindow = NULL;
	SDL_Quit();
}

/**
 * @brief Return a list which keys are currently pressed down. Not all keys are tracked; only the specific
 * ones which are checked inside the body of this function! To each key an unsigned char is assigned. For 
 * the arrow keys, the chars 'L', 'R', 'U', 'D' are used (the L,R,U and D keys are not monitored).
 * @return std::vector<unsigned char>
 */
std::vector<unsigned char> KeyboardInput_SDL::WhichKeysDown()
{
    //Event handler
    SDL_Event e;
    std::vector<unsigned char> KeysDown;

    //Handle events on queue
    while( SDL_PollEvent( &e ) != 0 ) // Required to update KeyboardState
    {
        //User requests quit
        if( e.type == SDL_QUIT )
        {
			quit = true;
        }
    }

    // Add any keys that you also want to capture.
    // For simplicity, I have given all arrow keys a character code (an enum would have been better here...)
    const Uint8* currentKeyStates = SDL_GetKeyboardState( NULL );
    if ( currentKeyStates[ SDL_SCANCODE_UP ] ) KeysDown.push_back ('U');
    if ( currentKeyStates[ SDL_SCANCODE_DOWN ] ) KeysDown.push_back ('D');
    if ( currentKeyStates[ SDL_SCANCODE_LEFT ] ) KeysDown.push_back ('L');
    if ( currentKeyStates[ SDL_SCANCODE_RIGHT ] ) KeysDown.push_back ('R');
	if ( currentKeyStates[ SDL_SCANCODE_Q ] ) KeysDown.push_back ('Q');
    return KeysDown;
}

bool KeyboardInput_SDL::get_quit()
{
	return quit;
}
