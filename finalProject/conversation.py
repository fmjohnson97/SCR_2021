import random

ENDINGS=['no', 'nope','bye','stop','goodbye','never mind']
CONFIRMS=['yes','yeah','absolutely']
MISTAKE_THRESHOLD=.3

def getGoalFromHuman(dummy_list):
    # location= random.choice(list(dummy_list.keys()))
    # import pdb; pdb.set_trace()
    # greet the user and get the goal location
    response=input('Greetings! Where would you like to go?')
    # account for the human typing somewhere not in the env
    while response.lower() not in list(dummy_list.keys()):
        print("I'm sorry. I don't know where that is.")
        response=input('Please choose another location.')
        # end the interaction if the human wants to leave
        if response in ENDINGS:
            print('Goodbye!')
            return None

    # determine if we've heard them properly; if not, change the heard location to something else
    p_heard=random.random()
    if p_heard<MISTAKE_THRESHOLD:
        response=random.choice(list(dummy_list.keys()))

    # make sure you have the correct destination
    confirmation=input("You'd like to go to "+response+'?')
    if confirmation in ENDINGS:
        # if not, get the goal location again
        response = input('Looks like I misheard you. Can you tell me your destination again?')
        # account for the human typing somewhere not in the env
        while response.lower() not in list(dummy_list.keys()):
            print("I'm sorry. I don't know where that is.")
            response = input('Please choose another location.')
            # end the interaction if the human wants to leave
            if response in ENDINGS:
                print('Goodbye!')
                return None
    print("Great. Let's go to "+response)
    return response

def getPathPreferences():
    # ask if they need accomodations
    response = input("Do you have any special mobility accomodations?")
    if response in ENDINGS:
        # if not then plan path
        print('Ok. Planning your path now.')
        return None
    # get the obstacles to avoid
    restrictions=input('Ok. What would you like to avoid?')
    if response in ENDINGS:
        # if the human changes their mind then just plan the path
        print('Ok. Planning your path now.')
        return None
    # list obstacles that the user wants to avoid
    print('Great. Planning a path now that avoids',restrictions)
    restrictions=restrictions.split(',')
    return restrictions




dummy_list = {'plant':None,
                  'motorcycle':None,
                  'r2d2':None,
                  'bathroom':None,
                  'blue chair':None,
                  'yellow chair':None,
                  'table':None}

# get location from user
loc=getGoalFromHuman(dummy_list)
# get obstacles to avoid from user
restrictions=getPathPreferences()
# get leading preference from the user
lead=input('Would you like me to take you there?')
# make sure the user enters yes or no type responses
while lead.lower() not in ENDINGS and lead.lower() not in CONFIRMS:
    print("I'm sorry. I don't understand your response.")
    lead = input('Would you like me to take you there?')

if lead in ENDINGS:
    print('Ok. Goodbye!')
else:
    print("Ok. Let's go!")