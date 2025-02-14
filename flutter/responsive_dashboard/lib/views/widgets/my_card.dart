import 'package:flutter/material.dart';
import 'package:flutter_svg/flutter_svg.dart';
import 'package:flutter_svg/svg.dart';
import 'package:responsive_dashboard/utils/app_images.dart';
import 'package:responsive_dashboard/utils/app_styles.dart';

class MyCard extends StatelessWidget {
  const MyCard({super.key});

  @override
  Widget build(BuildContext context) {
    return Column(
      children: [
        const SizedBox(
          height: 20,
        ),
        AspectRatio(
          aspectRatio: 420 / 215,
          child: Container(
            decoration: ShapeDecoration(
              image: const DecorationImage(
                fit: BoxFit.fill,
                image: AssetImage(
                  Assets.imagesCardBackground,
                ),
              ),
              color: const Color(0xFF4EB7F2),
              shape: RoundedRectangleBorder(
                borderRadius: BorderRadius.circular(12),
              ),
            ),
            child: Padding(
              padding: const EdgeInsets.symmetric(
                horizontal: 16,
                vertical: 4,
              ),
              child: Column(
                children: [
                  ListTile(
                    contentPadding: const EdgeInsets.all(
                      10,
                    ),
                    title: Text(
                      'Card Title',
                      style: AppStyles.styleRegular16(context)
                          .copyWith(color: Colors.white),
                    ),
                    subtitle:  Text(
                      'Syah Bandi',
                      style: AppStyles.styleMedium20(context),
                    ),
                    trailing: SvgPicture.asset(
                      Assets.imagesGallery,
                    ),
                  ),
                  const Spacer(),
                  Row(
                    mainAxisAlignment: MainAxisAlignment.end,
                    children: [
                      Text(
                        '0918 8124 0042 8129',
                        style: AppStyles.styleSemiBold24(context).copyWith(
                          color: Colors.white,
                        ),
                      )
                    ],
                  ),
                  const SizedBox(
                    height: 12,
                  ),
                  Row(
                    mainAxisAlignment: MainAxisAlignment.end,
                    children: [
                      Text(
                        '12/20',
                        style: AppStyles.styleRegular16(context).copyWith(
                          color: Colors.white,
                        ),
                      ),
                      const SizedBox(
                        width: 6,
                      ),
                      Text(
                        '-',
                        style: AppStyles.styleRegular16(context).copyWith(
                          color: Colors.white,
                        ),
                      ),
                      const SizedBox(
                        width: 6,
                      ),
                      Text(
                        '124',
                        style: AppStyles.styleRegular16(context).copyWith(
                          color: Colors.white,
                        ),
                      ),
                    ],
                  ),
                  const SizedBox(
                    height: 16,
                  ),
                ],
              ),
            ),
          ),
        ),
      ],
    );
  }
}
