from API import *

try:
    while(1):
        #이곳에 코드를 작성하시오
        
        sleep(0.001)
        
        
        
        
except (KeyboardInterrupt,Exception) as err: # 건들지 마시오!
    if(type(err) != KeyboardInterrupt):
        print('오류 발생:', err)
    init()
    sys.exit()