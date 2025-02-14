import 'package:flutter/material.dart';
import 'package:flutter_svg/svg.dart';

class AllExpensessItemHeader extends StatelessWidget {
  const AllExpensessItemHeader({
    super.key,
    required this.image,
    this.imageBackground = const Color(0xFFFAFAFA),
    this.imageColor,
  });

  final String image;
  final Color? imageBackground, imageColor;
  @override
  Widget build(BuildContext context) {
    return Row(
      children: [
        Flexible(
          child: ConstrainedBox(
            constraints: const BoxConstraints(maxWidth: 60),
            child: AspectRatio(
              aspectRatio: 1,
              child: Container(
                decoration: ShapeDecoration(
                  color: imageBackground ??
                      Colors.white.withOpacity(0.10000000149011612),
                  shape: const OvalBorder(),
                ),
                child: Center(
                  child: SvgPicture.asset(
                    image,
                    colorFilter: ColorFilter.mode(
                        imageColor ?? const Color(0xff4EB7F2), BlendMode.srcIn),
                  ),
                ),
              ),
            ),
          ),
        ),
        const Spacer(),
        Transform.rotate(
          angle: -90 * 3.14 * 2 / 180,
          child: Icon(
            Icons.arrow_back_ios_new_outlined,
            size: 16,
            color: imageColor == null ? const Color(0xFF064061) : Colors.white,
          ),
        ),
      ],
    );
  }
}
