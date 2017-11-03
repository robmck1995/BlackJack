from errors import CardError, DuplicateCardError
class Deck:

    suits = ['Diamonds', 'Hearts', 'Clubs', 'Spades']

    # 1 is Ace, 11 is Jack, 12 is Queen, 13 is King
    numbers = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13]

    """ Deck is represented as a dictionary, 0 if the card
        is not present, 1 if it is

        cards is a list of cards
    """
    def __init__(self, cards):

        """ We create all of the cards in the deck and add
            them to the dictionary, setting it to 1 if the
            card was passed in as the list of cards, 0
            otherwise
        """
        self.deck = {}
        all_cards = self.gen_all_cards()
        for card in all_cards:
            if card in cards:
                self.add_card(card) 
            else:
                self.deck[card] = 0
        for i in range(len(cards)):
            card1 = cards[i]
            for j in range(i+1, len(cards)):
                card2 = cards[j]
                if card1 == card2:
                    print "Warning: Duplicate Cards in Input Deck"
                

    def gen_all_cards(self):
        cards = []
        for s in self.suits:
            for num in self.numbers:
                card = Card(s, num)
                cards.append(card)
        return cards
        
    def add_card(self, card):
        if card not in self.deck:
            self.deck[card] = 0
        if self.deck[card] == 0:
            self.deck[card] = 1
        else:
            message = "A " + str(card) + " is already present in the deck."
            raise DuplicateCardError(message)

    def __str__(self):
        s = "Cards:\n"
        for card in self.deck.keys():
            if self.deck[card] == 1:
                s += str(card) + "\n"
        s = s[:-1]
        return s

class Card:

    suits = ['Hearts', 'Diamonds', 'Spades', 'Clubs']
    def __init__(self, suit, number):
        if suit not in self.suits:
            raise CardError('Invalid Suit')
        if number not in range(1,14):
            raise CardError('Invalid Value')
        self.suit = suit
        self.number = number

    def __str__(self):
        value = ''
        if self.number == 1:
            value = 'Ace'
        elif self.number == 2:
            value = 'Two'
        elif self.number == 3:
            value = 'Three'
        elif self.number == 4:
            value = 'Four'
        elif self.number == 5:
            value = 'Five'
        elif self.number == 6:
            value = 'Six'
        elif self.number == 7:
            value = 'Seven'
        elif self.number == 8:
            value = 'Eight'
        elif self.number == 9:
            value = 'Nine'
        elif self.number == 10:
            value = 'Ten'
        elif self.number == 11:
            value = 'Jack'
        elif self.number == 12:
            value = 'Queen'
        elif self.number == 13:
            value = 'Jack'
        
        return value + ' of ' + self.suit

    def __hash__(self):
        multi = 0
        if self.suit == 'Hearts':
            multi = 1
        elif self.suit == 'Clubs':
            multi = 2
        elif self.suit == 'Spades':
            multi = 3
        return 13*multi+self.number-1

    def __eq__(self, other):
        return self.suit == other.suit and self.number == other.number
