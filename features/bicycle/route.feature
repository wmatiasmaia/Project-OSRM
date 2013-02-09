@routing @bicycle @route
Feature: Bike -  Test route parsing

    Background:
        Given the profile "bicycle"

    Scenario: Bike - Prefer routes 
        Given the node map
        | a |  |   |   |  | b |
        |   |  | c | d |  |   |

        And the ways
        | nodes |
        | ab    |
        | ac    |
        | cd    |
        | db    |

        And the relations
        | type  | route   | name        | network | way:route |
        | route | bicycle | Green Route | lcn     | ac,cd,db  |

        When I route I should get
        | from | to | route    |
        | a    | b  | ac,cd,db |

    Scenario: Bike - Use route name for unnamed ways 
        Given the node map
        | a |  |   |   |  | b |
        |   |  | c | d |  |   |

        And the ways
        | nodes | name  |
        | ac    | ac    |
        | cd    | (nil) |
        | db    | db    |

        And the relations
        | type  | route   | name        | network | way:route |
        | route | bicycle | Green Route | lcn     | ac,cd,db  |

        When I route I should get
        | from | to | route             |
        | a    | b  | ac,Green Route,db |
