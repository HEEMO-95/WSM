import pygame
import time
import numpy as np
lat,lon,elev,tilt,pan,distance,bearing,psi,lat1,lon1,elev1 = 0,0,0,0,0,0,0,0,0,0,0
msg1, msg2, msg3 = None, None, None  # Initialize global variables

def game():

    global lat,lon,elev,tilt,pan,distance,bearing,psi,lat1,lon1,elev1  # inputs
    global msg1, msg2, msg3  # Declare them as global within the module
    running = True
    mode = "SLAVE"
    # Initialize Pygame
    pygame.init()
    _circle_cache = {}
    def _circlepoints(r):
        r = int(round(r))
        if r in _circle_cache:
            return _circle_cache[r]
        x, y, e = r, 0, 1 - r
        _circle_cache[r] = points = []
        while x >= y:
            points.append((x, y))
            y += 1
            if e < 0:
                e += 2 * y - 1
            else:
                x -= 1
                e += 2 * (y - x) - 1
        points += [(y, x) for x, y in points if x > y]
        points += [(-x, y) for x, y in points if x]
        points += [(x, -y) for x, y in points if y]
        points.sort()
        return points

    def render(text, font, gfcolor=pygame.Color(255, 255, 255), ocolor=(0, 0, 0), opx=2):
        textsurface = font.render(text, True, gfcolor).convert_alpha()
        w = textsurface.get_width() + 2 * opx
        h = font.get_height()

        osurf = pygame.Surface((w, h + 2 * opx)).convert_alpha()
        osurf.fill((0, 0, 0, 0))

        surf = osurf.copy()

        osurf.blit(font.render(text, True, ocolor).convert_alpha(), (0, 0))

        for dx, dy in _circlepoints(opx):
            surf.blit(osurf, (dx + opx, dy + opx))

        surf.blit(textsurface, (opx, opx))
        return surf

    def decimal_degrees_to_dms(Decimal,lat:bool = False, lon:bool = False ):
        d = int(Decimal)
        m = int((Decimal - d) * 60)
        s = round((Decimal - d - m/60) * 3600.00,2)

        if lat == True:
            if d < 0:
                H = 'S'
            else:
                H = 'N'

        if lon == True:
            if d < 0:
                H = 'W'
            else:
                H = 'E'
        
        return (f"{abs(d)}°{abs(m)}'{abs(s)}"f'"{H}')


    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    screen = pygame.display.set_mode((1280, 720))
    
    while running:
        for event in pygame.event.get():

            if event.type == pygame.QUIT:
                running = False
                

            if event.type == pygame.JOYBUTTONDOWN:

                if event.button == 0:  # ✖
                    print('✖')
                    mode = 'DESIGNATE'

                if event.button == 1:  # ●
                    mode = 'CENTER'
                    print('●')

                if event.button == 2:  # ▲ 
                    mode = 'SLAVE'
                    print('▲')
                    
                if event.button == 3:  # ■
                    print('■')
                    mode = 'LOCK'
                    joystick.rumble(0, 1, 100)


        msg2 = round(joystick.get_axis(0), 1)
        msg3 = round(joystick.get_axis(1), 1)

        screen.fill((0, 0, 0))

        if mode == 'CENTER':
            msg1 = 1
            c_color, g_color, m_color = '#808080', '#808080', '#FFA500'

        if mode == 'SLAVE':
            msg1 = 2
            c_color = g_color = m_color = '#7FFFD4'

        if mode == 'DESIGNATE':
            msg1 = 3
            c_color, g_color, m_color = '#7FFFD4', '#FFA500', '#00FF00'
            
        if mode == 'LOCK':
            msg1 = 4
            c_color, g_color, m_color = '#FFA500', '#808080', '#FF0000'

        if bearing < 0:
            bearing = 360+bearing
    
        angle = np.radians(tilt)

        center_x = 250+320
        center_y = 40
        radius = 30
        
        
        font = pygame.font.SysFont('Tektur',bold=False, size=30)

        pygame.draw.rect(screen, '#FFFFFF', (250+375, 5, 30, 10))

        pygame.draw.circle(screen, '#FFFFFF', (center_x, center_y), radius, 3)
        end_x = center_x + int(np.sin(angle) * radius)
        end_y = center_y - int(np.cos(angle) * radius)

        pygame.draw.line(screen, '#FFFF00', (center_x, center_y), (end_x, end_y), 2)


        center_x1 = 250+390
        center_y1 = 40
        radius1 = 30

        angle1 = np.radians(pan+90)
        end_x1 = center_x1 + int(np.sin(angle1) * radius1)
        end_y1 = center_y1 - int(np.cos(angle1) * radius1)
        pygame.draw.circle(screen, '#FFFFFF', (center_x1, center_y1), radius, 3)
        pygame.draw.line(screen, '#FFFF00', (center_x1, center_y1), (end_x1, end_y1), 2)

        nfont = pygame.font.Font(None, 20)
        nc_x = center_x + int(np.sin(-psi) * radius) - 4
        nc_y = center_y - int(np.cos(-psi) * radius) - 6

        lat_DMS = decimal_degrees_to_dms(lat, lat=True)
        lon_DMS = decimal_degrees_to_dms(lon, lon=True)

        lat1_DMS = decimal_degrees_to_dms(lat1, lat=True)
        lon1_DMS = decimal_degrees_to_dms(lon1, lon=True)
        
        try:
            screen.blit(render(f"Mode:", font), (10, 10))

            screen.blit(render(f"Target Location:", font),
                        (30, 60))
            screen.blit(render(f"{mode}", font, gfcolor=pygame.Color(m_color)),
                        (100, 10))
            screen.blit(render(f"Lat: {lat_DMS}", font, gfcolor=pygame.Color(c_color)),
                        (50, 110))
            screen.blit(render(f"Lon: {lon_DMS}", font, gfcolor=pygame.Color(c_color)),
                        (50, 160))
            screen.blit(render(f"Elevation: {int(elev)}m", font, gfcolor=pygame.Color(c_color)),
                        (50, 210))
            screen.blit(render(f"Gimbal: {round(tilt,2)}°, {round(pan,2)}°", font, gfcolor=pygame.Color(g_color)),
                        (50, 260))
            screen.blit(render(f"Range:{round(distance,1)}m, Bearing:{round(bearing,1)}°", font,gfcolor=pygame.Color("#FFFF00")),
                        (30, 310))

            screen.blit(render("Aicraft Location:", font),
                        (50, 360))
            screen.blit(render(f"Lat: {lon1_DMS}", font),
                        (50, 410))
            screen.blit(render(f"Lon: {lat1_DMS}", font),
                        (50, 460))
            screen.blit(render(f"Elev: {round(elev1,1)}m", font),
                        (50, 510))

            north = nfont.render("N", True,'#FF0000')
            screen.blit(north, (nc_x, nc_y))
            

        finally:
            pygame.display.flip()
        

    msg1, msg2, msg3 = None,None,None
    pygame.quit()
