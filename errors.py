#Base Class for Errors
class Error(Exception):
    pass

#Error raised if duplicate cards added to deck
class DuplicateCardError(Error):

    def __init__(self, message):
        super(DuplicateCardError, self).__init__(message)
        self.message = message

# Error for trying to create an invalid card
class CardError(Error):

    def __init__(self, message):
        super(CardError, self).__init__(message)
        self.message = message
