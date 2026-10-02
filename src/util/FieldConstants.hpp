#pragma once

// Standard field geometry and named landmarks used by motion strategy.

#include <util/util.hpp>

// All dimensions and positions are in millimetres. The origin is at the field
// centre, with +x pointing right and +y pointing toward the opponent goal.
class FieldConstants {
    public:
        // Regulation dimensions.
        static constexpr float fieldLength = 2430.0f;
        static constexpr float fieldWidth = 1820.0f;
        static constexpr float boundaryInset = 250.0f;
        static constexpr float boundaryLineWidth = 50.0f;
        static constexpr float goalBoxDepth = 300.0f;
        static constexpr float goalBoxWidth = 900.0f;
        static constexpr float goalDepth = 74.0f;
        static constexpr float goalWidth = 450.0f;
        static constexpr float centreMarkSpacing = 300.0f;

        // Outer field corners.
        static constexpr Position2D fieldBottomLeft = Position2D(-910.0f, -1215.0f);
        static constexpr Position2D fieldBottomRight = Position2D(910.0f, -1215.0f);
        static constexpr Position2D fieldTopLeft = Position2D(-910.0f, 1215.0f);
        static constexpr Position2D fieldTopRight = Position2D(910.0f, 1215.0f);

        // Corners at the inside edge of the 50 mm white boundary line.
        static constexpr Position2D boundaryBottomLeft = Position2D(-610.0f, -915.0f);
        static constexpr Position2D boundaryBottomRight = Position2D(610.0f, -915.0f);
        static constexpr Position2D boundaryTopLeft = Position2D(-610.0f, 915.0f);
        static constexpr Position2D boundaryTopRight = Position2D(610.0f, 915.0f);

        // Friendly goal box corners.
        static constexpr Position2D friendlyGoalBoxBottomLeft = Position2D(-450.0f, -915.0f);
        static constexpr Position2D friendlyGoalBoxBottomRight = Position2D(450.0f, -915.0f);
        static constexpr Position2D friendlyGoalBoxTopLeft = Position2D(-450.0f, -615.0f);
        static constexpr Position2D friendlyGoalBoxTopRight = Position2D(450.0f, -615.0f);
        static constexpr Position2D friendlyGoalBoxPosition = Position2D(0.0f, -765.0f);

        // Opponent goal box corners.
        static constexpr Position2D opponentGoalBoxBottomLeft = Position2D(-450.0f, 615.0f);
        static constexpr Position2D opponentGoalBoxBottomRight = Position2D(450.0f, 615.0f);
        static constexpr Position2D opponentGoalBoxTopLeft = Position2D(-450.0f, 915.0f);
        static constexpr Position2D opponentGoalBoxTopRight = Position2D(450.0f, 915.0f);
        static constexpr Position2D opponentGoalBoxPosition = Position2D(0.0f, 765.0f);

        // Goal centres, posts, and rear corners.
        static constexpr Position2D friendlyGoalPosition = Position2D(0.0f, -915.0f);
        static constexpr Position2D friendlyGoalLeftPost = Position2D(-225.0f, -915.0f);
        static constexpr Position2D friendlyGoalRightPost = Position2D(225.0f, -915.0f);
        static constexpr Position2D friendlyGoalBackLeft = Position2D(-225.0f, -989.0f);
        static constexpr Position2D friendlyGoalBackRight = Position2D(225.0f, -989.0f);

        static constexpr Position2D opponentGoalPosition = Position2D(0.0f, 915.0f);
        static constexpr Position2D opponentGoalLeftPost = Position2D(-225.0f, 915.0f);
        static constexpr Position2D opponentGoalRightPost = Position2D(225.0f, 915.0f);
        static constexpr Position2D opponentGoalBackLeft = Position2D(-225.0f, 989.0f);
        static constexpr Position2D opponentGoalBackRight = Position2D(225.0f, 989.0f);

        // Centre and the two marks spaced 300 mm across the field.
        static constexpr Position2D centre = Position2D(0.0f, 0.0f);
        static constexpr Position2D centreLeftMark = Position2D(-300.0f, 0.0f);
        static constexpr Position2D centreRightMark = Position2D(300.0f, 0.0f);
};
