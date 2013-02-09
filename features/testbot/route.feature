@routing @testbot @route
Feature: Testbot -  Test route parsing

    Background:
        Given the profile "testbot"

    Scenario: Testbot -  Route types
    Testbot doubles the speed on routes with route=bot, and ignores other routes
        Given the node map
        | a | b | c | d |

        And the ways
        | nodes |
        | ab    |
        | bc    |
        | cd    |

        And the relations
        | type  | route   | way:route |
        | route | bot     | bc        |
        | route | bicycle | cd        |

        When I route I should get
        | from | to | route | distance | time    |
        | a    | b  | ab    | 100m +-1 | 10s +-1 |
        | b    | c  | bc    | 100m +-1 | 5s +-1  |
        | c    | d  | cd    | 100m +-1 | 10s +-1 |

    Scenario: Testbot -  Route tags
    Testbot quadruples the speed on routes with style=turbo
        Given the node map
        | a | b | c |

        And the ways
        | nodes |
        | ab    |
        | bc    |

        And the relations
        | type  | route | style | way:route |
        | route | bot   |       | ab        |
        | route | bot   | turbo | bc        |

        When I route I should get
        | from | to | route | distance | time   |
        | b    | a  | ab    | 100m +-1 | 5s +-1 |
        | b    | c  | bc    | 100m +-1 | 2s +-1 |

    Scenario: Testbot - Overlapping routes
    Testbot multiplies the speed gain of overlapping routes
        Given the node map
        | a | b | c |
        | d | e | f |

        And the ways
        | nodes |
        | ab    |
        | bc    |
        | de    |
        | ef    |
        | be    |

        And the relations
        | type  | route | way:route |
        | route | bot   | ab,be,ef  |
        | route | bot   | de,be,bc  |

        When I route I should get
        | from | to | route | distance | time   |
        | a    | b  | ab    | 100m +-1 | 5s +-1 |
        | b    | c  | bc    | 100m +-1 | 5s +-1 |
        | d    | e  | de    | 100m +-1 | 5s +-1 |
        | e    | f  | ef    | 100m +-1 | 5s +-1 |
        | b    | e  | be    | 100m +-1 | 2s +-1 |
        | e    | b  | be    | 100m +-1 | 2s +-1 |
    
    Scenario: Testbot -  Route with borward/backward roles
    Testbot only accelerates when moving in the direction of the route
        Given the node map
        | a | b | c |

        And the ways
        | nodes |
        | ab    |
        | bc    |

        And the relations
        | type  | route | way:forward | way:backward |
        | route | bot   | ab          |              |
        | route | bot   |             | bc           |

        When I route I should get
        | from | to | route | distance | time    |
        | a    | b  | ab    | 100m +-1 | 5s +-1  |
        | b    | a  | ab    | 100m +-1 | 10s +-1 |
        | b    | c  | bc    | 100m +-1 | 10s +-1 |
        | c    | b  | bc    | 100m +-1 | 5s +-1  |
    
    Scenario: Testbot -  Prefer routes 
        Given the node map
        | a |  |  | b |
        | c |  |  | d |

        And the ways
        | nodes |
        | ab    |
        | ac    |
        | cd    |
        | db    |

        And the relations
        | type  | route | way:route |
        | route | bot   | ac,cd,db  |

        When I route I should get
        | from | to | route    |
        | a    | b  | ac,cd,db |