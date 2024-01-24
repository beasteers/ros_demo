from typing import List
import strawberry

@strawberry.type
class Recipe:
    name: str
    tools: List[str]
    ingredients: List[str]
    instructions: List[str]

@strawberry.type
class Query:
    recipes = List[Recipe]


schema = strawberry.Schema(query=Query)
